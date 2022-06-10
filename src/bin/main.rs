#![no_main]
#![no_std]

use stellantis_can_adapter as _; // global logger + panicking-behavior + memory layout

use bxcan::{filter::Mask32, Data, Fifo, Frame, Interrupts, Rx0, StandardId, Tx};
use heapless::binary_heap::{BinaryHeap, Max};

use stm32f1xx_hal::{
    can::Can,
    pac::{Interrupt, CAN1, CAN2, TIM2},
    prelude::*,
    rtc::{Rtc, RtcClkLsi},
    timer::monotonic::MonoTimer,
};

mod handlers;
use handlers::*;

#[derive(Debug)]
pub struct RawFrame(Frame);

/// Ordering is based on the Identifier and frame type (data vs. remote) and can be used to sort
/// frames by priority.
impl Ord for RawFrame {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.0.priority().cmp(&other.0.priority())
    }
}

impl PartialOrd for RawFrame {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for RawFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == core::cmp::Ordering::Equal
    }
}

impl Eq for RawFrame {}

/// Shared datetime clock state.
pub struct ClockState {
    /// The RTC of the chip we are running on.
    rtc: Rtc<RtcClkLsi>,
    /// Format of the time clock, ie: 12/24 hour format.
    clock_format: canpsa::config::ClockFormat,
    /// Clock display mode.
    clock_disp_mode: canpsa::config::DisplayMode,
}

/// Shared vehicle state.
#[derive(Copy, Clone)]
pub struct VehicleState {
    /// Economy mode flag.
    economy_mode: bool,
    /// Vehicle main status.
    main_status: canpsa::vehicle::MainStatus,
    /// Generator working flag.
    generator_working: bool,
    /// Cockpit steering wheel position,
    /// used to determine which front door is the driver's door.
    steering_wheel_position: canpsa::vehicle::SteeringWheelPosition,
    /// Front left door opene flag.
    front_left_door_open: bool,
    /// Front right door open flag.
    front_right_door_open: bool,
    /// Rear left door open flag.
    rear_left_door_open: bool,
    /// Rear right door open flag.
    rear_right_door_open: bool,
    /// Boot open flag.
    boot_open: bool,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            economy_mode: false,
            main_status: canpsa::vehicle::MainStatus::Off,
            generator_working: false,
            steering_wheel_position: canpsa::vehicle::SteeringWheelPosition::Left,
            front_left_door_open: false,
            front_right_door_open: false,
            rear_left_door_open: false,
            rear_right_door_open: false,
            boot_open: false,
        }
    }
}

/// Shared display config (units, language, ...)
/// These parameters are saved into flash memory.
#[derive(Copy, Clone)]
pub struct DisplayConfig {
    /// Language used for display.
    language: canpsa::config::Language,
    /// Consumption unit used for display.
    consumption_unit: canpsa::config::ConsumptionUnit,
    /// Distance unit used for display.
    distance_unit: canpsa::config::DistanceUnit,
    /// Temperature unit used for display.
    temperature_unit: canpsa::config::TemperatureUnit,
    /// Volume unit used for display.
    volume_unit: canpsa::config::VolumeUnit,
}

impl Default for DisplayConfig {
    fn default() -> Self {
        Self {
            language: canpsa::config::Language::English,
            consumption_unit: canpsa::config::ConsumptionUnit::VolumePerDistance,
            distance_unit: canpsa::config::DistanceUnit::Kilometer,
            temperature_unit: canpsa::config::TemperatureUnit::Celsius,
            volume_unit: canpsa::config::VolumeUnit::Liter,
        }
    }
}

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[local]
    struct Local {
        can_1_tx: Tx<Can<CAN1>>,
        can_1_rx: Rx0<Can<CAN1>>,
        can_2_tx: Tx<Can<CAN2>>,
        can_2_rx: Rx0<Can<CAN2>>,
    }
    #[shared]
    struct Shared {
        clock: ClockState,
        can_1_tx_queue: BinaryHeap<RawFrame, Max, 16>,
        can_2_tx_queue: BinaryHeap<RawFrame, Max, 16>,
        vehicle_state: VehicleState,
        display_config: DisplayConfig,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MyMono = MonoTimer<TIM2, 1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Init clocks.
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).freeze(&mut flash.acr);

        // Monotonic timer.
        let mono = cx.device.TIM2.monotonic(&clocks);

        // RTC config. Use LSI clock because we don't have an external 32.768 kHz crystal.
        let mut pwr = cx.device.PWR;
        let mut backup_domain = rcc.bkp.constrain(cx.device.BKP, &mut pwr);
        let rtc = Rtc::new_lsi(cx.device.RTC, &mut backup_domain);

        // CAN peripherals config.
        let mut afio = cx.device.AFIO.constrain();
        let can_1_peripheral = Can::new(cx.device.CAN1);
        let can_2_peripheral = Can::new(cx.device.CAN2);

        // Select pins for CAN1 peripheral.
        let mut gpio_a = cx.device.GPIOA.split();
        let can_1_rx_pin = gpio_a.pa11.into_floating_input(&mut gpio_a.crh);
        let can_1_tx_pin = gpio_a.pa12.into_alternate_push_pull(&mut gpio_a.crh);
        can_1_peripheral.assign_pins((can_1_tx_pin, can_1_rx_pin), &mut afio.mapr);

        // Select pins for CAN2 peripheral.
        let mut gpio_b = cx.device.GPIOB.split();
        let can_2_rx_pin = gpio_b.pb5.into_floating_input(&mut gpio_b.crl);
        let can_2_tx_pin = gpio_b.pb6.into_alternate_push_pull(&mut gpio_b.crl);
        can_2_peripheral.assign_pins((can_2_tx_pin, can_2_rx_pin), &mut afio.mapr);

        // Set CAN1 speed.
        let mut can_1 = bxcan::Can::builder(can_1_peripheral)
            .set_bit_timing(0x00070013) // APB1 (PCLK1): 25MHz, Bit rate: 125kBit/s, Sample Point 87.5% - Value was calculated with http://www.bittiming.can-wiki.info/
            .leave_disabled();

        // Set CAN2 speed.
        let mut can_2 = bxcan::Can::builder(can_2_peripheral)
            .set_bit_timing(0x00070013) // APB1 (PCLK1): 25MHz, Bit rate: 125kBit/s, Sample Point 87.5% - Value was calculated with http://www.bittiming.can-wiki.info/
            .leave_disabled();

        // Set CAN1 and CAN2 filters to accept everything.
        can_1
            .modify_filters()
            .set_split(14)
            .clear()
            .enable_bank(0, Fifo::Fifo0, Mask32::accept_all())
            .slave_filters()
            .clear()
            .enable_bank(14, Fifo::Fifo0, Mask32::accept_all());

        // Set CAN1 and CAN2 interrupts.
        can_1.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        can_2.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );

        // Enable CAN1 and CAN2 peripherals.
        nb::block!(can_1.enable_non_blocking()).unwrap();
        nb::block!(can_2.enable_non_blocking()).unwrap();

        let (can_1_tx, can_1_rx, _) = can_1.split();
        let (can_2_tx, can_2_rx, _) = can_2.split();

        // Start tasks.
        send_2004_0x228::spawn().unwrap();
        send_2010_0x276::spawn().unwrap();
        send_2010_0x122::spawn().unwrap();

        (
            Shared {
                clock: ClockState {
                    rtc,
                    clock_format: canpsa::config::ClockFormat::H24,
                    clock_disp_mode: canpsa::config::DisplayMode::Blinking,
                },
                can_1_tx_queue: BinaryHeap::new(),
                can_2_tx_queue: BinaryHeap::new(),
                vehicle_state: VehicleState::default(),
                display_config: DisplayConfig::default(),
            },
            Local {
                can_1_tx,
                can_1_rx,
                can_2_tx,
                can_2_rx,
            },
            init::Monotonics(mono),
        )
    }

    /// AEE2004 0x228 frame sending task.
    #[task(shared = [can_1_tx_queue, clock])]
    fn send_2004_0x228(mut cx: send_2004_0x228::Context) {
        let clock_time = cx.shared.clock.lock(|clk| clk.rtc.current_time());

        let offset_datetime = time::OffsetDateTime::from_unix_timestamp(
            clock_time as i64 + canpsa::UNIX_EPOCH_OFFSET,
        )
        .unwrap();

        let x228_frame_repr = canpsa::aee2004::conf::x228::Repr {
            time: offset_datetime.time(),
        };

        let mut x228_buf = [0; canpsa::aee2004::conf::x228::FRAME_LEN];
        let mut x228_frame = canpsa::aee2004::conf::x228::Frame::new_unchecked(&mut x228_buf);
        x228_frame_repr.emit(&mut x228_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x228).unwrap(),
            Data::new(&x228_buf).unwrap(),
        );

        cx.shared.can_1_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::USB_HP_CAN_TX);
        });

        send_2004_0x228::spawn_after(60.secs()).unwrap();
    }

    /// AEE2010 0x122 frame sending task.
    #[task(shared = [can_2_tx_queue])]
    fn send_2010_0x122(mut cx: send_2010_0x122::Context) {
        let x122_frame_repr = canpsa::aee2010::infodiv::x122::Repr {
            front_panel_buttons_state: [false; 44],
            front_panel_bp_button_state: false,
            front_panel_esp_button_state: false,
            front_panel_first_wheel_sync_request: false,
            front_panel_second_wheel_sync_request: false,
            front_panel_first_wheel_ticks_counter: 0,
            front_panel_second_wheel_ticks_counter: 0,
        };

        let mut x122_buf = [0; canpsa::aee2010::infodiv::x122::FRAME_LEN];
        let mut x122_frame = canpsa::aee2010::infodiv::x122::Frame::new_unchecked(&mut x122_buf);
        x122_frame_repr.emit(&mut x122_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x122).unwrap(),
            Data::new(&x122_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });

        send_2010_0x122::spawn_after(200.millis()).unwrap();
    }

    /// AEE2010 0x276 frame sending task.
    #[task(shared = [can_2_tx_queue, clock])]
    fn send_2010_0x276(mut cx: send_2010_0x276::Context) {
        let clock_data = cx.shared.clock.lock(|clk| {
            (
                clk.rtc.current_time(),
                clk.clock_format,
                clk.clock_disp_mode,
            )
        });

        let offset_datetime = time::OffsetDateTime::from_unix_timestamp(
            clock_data.0 as i64 + canpsa::UNIX_EPOCH_OFFSET,
        )
        .unwrap();

        let x276_frame_repr = canpsa::aee2010::infodiv::x276::Repr {
            clock_format: clock_data.1,
            clock_disp_mode: clock_data.2,
            utc_datetime: offset_datetime,
            adblue_autonomy: 0xfffe, // Unavailable
            adblue_autonomy_display_request: false,
        };

        let mut x276_buf = [0; canpsa::aee2010::infodiv::x276::FRAME_LEN];
        let mut x276_frame = canpsa::aee2010::infodiv::x276::Frame::new_unchecked(&mut x276_buf);
        x276_frame_repr.emit(&mut x276_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x276).unwrap(),
            Data::new(&x276_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });

        send_2010_0x276::spawn_after(1.secs()).unwrap();
    }

    /// AEE2010 0x236 frame sending task.
    #[task(shared = [can_2_tx_queue, vehicle_state])]
    fn send_2010_x236(mut cx: send_2010_x236::Context) {
        let vehicle_state = cx.shared.vehicle_state.lock(|state| *state);

        let electrical_network_status =
            match (vehicle_state.generator_working, vehicle_state.economy_mode) {
                (true, true) => canpsa::vehicle::ElectricalNetworkState::GeneratorFailSoftMode,
                (true, false) => canpsa::vehicle::ElectricalNetworkState::GeneratorNormal,
                (false, true) => canpsa::vehicle::ElectricalNetworkState::BatteryFailSoftMode,
                (false, false) => canpsa::vehicle::ElectricalNetworkState::BatteryNormal,
            };

        let driver_door_open_evt = match (
            vehicle_state.steering_wheel_position,
            vehicle_state.front_left_door_open,
            vehicle_state.front_right_door_open,
        ) {
            (canpsa::vehicle::SteeringWheelPosition::Right, _, true) => true,
            (canpsa::vehicle::SteeringWheelPosition::Left, true, _) => true,
            _ => false,
        };

        let fault_log_context = match (vehicle_state.main_status, vehicle_state.economy_mode) {
            (canpsa::vehicle::MainStatus::Off, true) => {
                canpsa::vehicle::FaultLogContext::MainOffEco
            }
            (canpsa::vehicle::MainStatus::Off, false) => canpsa::vehicle::FaultLogContext::MainOff,
            (canpsa::vehicle::MainStatus::On, true) => canpsa::vehicle::FaultLogContext::MainOnEco,
            (canpsa::vehicle::MainStatus::Cranking, true) => {
                canpsa::vehicle::FaultLogContext::MainOnEco
            }
            _ => canpsa::vehicle::FaultLogContext::MainOn,
        };

        let x236_frame_repr = canpsa::aee2010::infodiv::x236::Repr {
            vehicle_config_mode: canpsa::vehicle::VehicleConfigMode::Customer,
            electrical_network_status,
            vsm_temporal_counter: 0xFFFFFFFE,
            fault_log_context,
            driver_door_open_evt,
            boot_open: vehicle_state.boot_open,
            gct_reset_counter: 0xfe,
            power_on_req_denied: false,
        };

        let mut x236_buf = [0; canpsa::aee2010::infodiv::x236::FRAME_LEN];
        let mut x236_frame = canpsa::aee2010::infodiv::x236::Frame::new_unchecked(&mut x236_buf);
        x236_frame_repr.emit(&mut x236_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x236).unwrap(),
            Data::new(&x236_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });

        send_2010_x236::spawn_after(500.millis()).unwrap();
    }

    /// Master AEE2004 CAN sending task.
    #[task(binds = USB_HP_CAN_TX, local = [can_1_tx], shared = [can_1_tx_queue])]
    fn can_1_tx(mut cx: can_1_tx::Context) {
        // Clear interrupts.
        cx.local.can_1_tx.clear_interrupt_flags();

        cx.shared.can_1_tx_queue.lock(|tx_queue| {
            while let Some(frame) = tx_queue.peek() {
                match cx.local.can_1_tx.transmit(&frame.0) {
                    Ok(status) => match status.dequeued_frame() {
                        None => {
                            // Frame was successfully placed into a transmit buffer.
                            tx_queue.pop();
                        }
                        Some(pending_frame) => {
                            // A lower priority frame was replaced with our high priority frame.
                            // Put the low priority frame back in the transmit queue.
                            tx_queue.pop();
                            tx_queue.push(RawFrame(pending_frame.clone())).unwrap();
                        }
                    },
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => unreachable!(),
                }
            }
        });
    }

    /// Master AEE2010 CAN sending task.
    #[task(binds = CAN2_TX, local = [can_2_tx], shared = [can_2_tx_queue])]
    fn can_2_tx(mut cx: can_2_tx::Context) {
        // Clear interrupts.
        cx.local.can_2_tx.clear_interrupt_flags();

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            while let Some(frame) = tx_queue.peek() {
                match cx.local.can_2_tx.transmit(&frame.0) {
                    Ok(status) => match status.dequeued_frame() {
                        None => {
                            // Frame was successfully placed into a transmit buffer.
                            tx_queue.pop();
                        }
                        Some(pending_frame) => {
                            // A lower priority frame was replaced with our high priority frame.
                            // Put the low priority frame back in the transmit queue.
                            tx_queue.pop();
                            tx_queue.push(RawFrame(pending_frame.clone())).unwrap();
                        }
                    },
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => unreachable!(),
                }
            }
        });
    }

    /// Receive from AEE2004 Conf CAN bus.
    #[task(binds = USB_LP_CAN_RX0, local = [can_1_rx], shared = [can_2_tx_queue, vehicle_state, display_config])]
    fn can_1_rx0(mut cx: can_1_rx0::Context) {
        loop {
            match cx.local.can_1_rx.receive() {
                Ok(recv_frame) => {
                    // Filter non-standard Id frames.
                    let rx_id = match recv_frame.id() {
                        bxcan::Id::Standard(id) => id.as_raw(),
                        _ => continue,
                    };

                    // Filter non Data frames.
                    let rx_data = match recv_frame.data() {
                        Some(data) => data,
                        None => continue,
                    };

                    defmt::info!("CAN AEE2004: received frame [{=u16:#x}]", rx_id);

                    // Dispatch frames to matching handlers.
                    let result = match rx_id {
                        0x036 => parse_2004_x036(rx_data).and_then(|repr| {
                            cx.shared.vehicle_state.lock(|state| {
                                state.economy_mode = repr.economy_mode_enabled;
                            });
                            Ok(HandlerDecision::Forward(recv_frame))
                        }),
                        0x0f6 => parse_2004_x0f6(rx_data).and_then(|repr| {
                            cx.shared.vehicle_state.lock(|state| {
                                state.generator_working = repr.generator_working;
                                state.main_status = repr.vehicle_main_status;
                                state.steering_wheel_position = repr.steering_wheel_position;
                            });
                            Ok(HandlerDecision::Forward(recv_frame))
                        }),
                        0x220 => parse_2004_x220(rx_data).and_then(|repr| {
                            cx.shared.vehicle_state.lock(|state| {
                                state.front_left_door_open = repr.front_left_door_opened;
                                state.front_right_door_open = repr.front_right_door_opened;
                                state.rear_left_door_open = repr.rear_left_door_opened;
                                state.rear_right_door_open = repr.rear_right_door_opened;
                            });
                            Ok(HandlerDecision::Ok)
                        }),
                        0x260 => parse_2004_x260(rx_data).and_then(|repr| {
                            let mut fwd_repr = canpsa::aee2010::infodiv::x260::Repr::from(&repr);

                            cx.shared.display_config.lock(|config| {
                                fwd_repr.consumption_unit = config.consumption_unit;
                                fwd_repr.distance_unit = config.distance_unit;
                                fwd_repr.language = config.language;
                                fwd_repr.temperature_unit = config.temperature_unit;
                                fwd_repr.volume_unit = config.volume_unit;
                            });

                            encode_2010_x260(&fwd_repr)
                        }),
                        0x128 => handle_2004_x128(rx_data),
                        0x168 => handle_2004_x168(rx_data),
                        0x1a8 => handle_2004_x1a8(rx_data),
                        0x1d0 => handle_2004_x1d0(rx_data),
                        0x1e1 => handle_2004_x1e1(rx_data),
                        0x261 => handle_2004_x261(rx_data),
                        0x2a1 => handle_2004_x2a1(rx_data),
                        0x361 => handle_2004_x361(rx_data),
                        0x3a7 => handle_2004_x3a7(rx_data),
                        0x228 => Ok(HandlerDecision::Ok), // Filter received 0x228 if this happens...
                        _ => {
                            // Forward frame as-is.
                            defmt::info!("CAN AEE2004: forward as-is [{=u16:#x}]", rx_id);
                            Ok(HandlerDecision::Forward(recv_frame))
                        }
                    };

                    match result {
                        Ok(HandlerDecision::Forward(frame)) => {
                            cx.shared.can_2_tx_queue.lock(|tx_queue| {
                                tx_queue.push(RawFrame(frame)).unwrap();
                                rtic::pend(Interrupt::CAN2_TX);
                            });
                        }
                        Ok(HandlerDecision::Ok) => {}
                        Err(_) => {}
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }

    /// Receive from AEE2010 Infodiv CAN bus.
    #[task(binds = CAN2_RX0, local = [can_2_rx], shared = [can_1_tx_queue, clock, display_config])]
    fn can_2_rx0(mut cx: can_2_rx0::Context) {
        loop {
            match cx.local.can_2_rx.receive() {
                Ok(recv_frame) => {
                    // Filter non-standard Id frames.
                    let rx_id = match recv_frame.id() {
                        bxcan::Id::Standard(id) => id.as_raw(),
                        _ => continue,
                    };

                    // Filter non Data frames.
                    let rx_data = match recv_frame.data() {
                        Some(data) => data,
                        None => continue,
                    };

                    defmt::info!("CAN AEE2010: received frame [{=u16:#x}]", rx_id);

                    let result = match rx_id {
                        0x15b => parse_2010_x15b(rx_data).and_then(|repr| {
                            let fwd_repr = canpsa::aee2004::conf::x15b::Repr::from(&repr);

                            if repr.units_language_parameters_validity {
                                cx.shared.display_config.lock(|config| {
                                    config.consumption_unit = repr.consumption_unit;
                                    config.distance_unit = repr.distance_unit;
                                    config.language = repr.language;
                                    config.temperature_unit = repr.temperature_unit;
                                    config.volume_unit = repr.volume_unit;
                                });
                            }
                            encode_2004_x15b(&fwd_repr)
                        }),
                        0x1a9 => parse_2010_x1a9(rx_data).and_then(|repr| {
                            let fwd_repr = canpsa::aee2004::conf::x167::Repr {
                                mfd_trip_computer_page: canpsa::mfd::TripComputerPage::Nothing,
                                maintenance_reset_request: false,
                                emergency_call_in_progress: false,
                                fault_recall_request: repr.fault_check_request,
                                trip_computer_secondary_trip_reset_request: repr.trip_computer_secondary_trip_reset_request,
                                trip_computer_primary_trip_reset_request: repr.trip_computer_primary_trip_reset_request,
                                pre_conditioning_time: 0,
                                telematics_enabled: repr.telematics_enabled,
                                black_panel_enabled: repr.black_panel_enabled,
                                indirect_under_inflation_reset_request: repr.indirect_under_inflation_button_state,
                                pre_conditioning_request: false,
                                total_trip_distance: 0xffff,
                                interactive_message: 0x7fff,
                                stop_and_start_button_state: repr.stop_start_button_state,
                                lane_centering_button_state: repr.lane_centering_button_state,
                                parking_sensors_button_state: repr.parking_sensors_button_state,
                                user_action_on_mfd: canpsa::mfd::UserAction2004::NoAction,
                                user_value: 0,
                            };
                            encode_2004_x167(&fwd_repr)
                        }),
                        0x39b => parse_2010_x39b(rx_data).and_then(|repr| {
                            cx.shared.clock.lock(|clk| {
                                let time_epoch = repr
                                    .utc_datetime
                                    .unix_timestamp()
                                    .saturating_sub(canpsa::UNIX_EPOCH_OFFSET)
                                    as u32;

                                clk.rtc.set_time(time_epoch);
                                clk.clock_format = repr.clock_format;
                                clk.clock_disp_mode = canpsa::config::DisplayMode::Steady;
                            });

                            Ok(HandlerDecision::Ok)
                        }),
                        0x1e5 => handle_2010_x1e5(rx_data),
                        0x224 => Ok(HandlerDecision::Ok), // Filter received 0x224
                        0x2e9 => Ok(HandlerDecision::Ok), // Filter received 0x2e9
                        0x328 => Ok(HandlerDecision::Ok), // Filter received 0x328
                        0x329 => Ok(HandlerDecision::Ok), // Filter received 0x329
                        0x364 => Ok(HandlerDecision::Ok), // Filter received 0x364
                        0x3a1 => Ok(HandlerDecision::Ok), // Filter received 0x3a1
                        _ => {
                            // Forward frame as-is.
                            defmt::info!("CAN AEE2010: forward as-is [{=u16:#x}]", rx_id);
                            Ok(HandlerDecision::Forward(recv_frame))
                        }
                    };

                    match result {
                        Ok(HandlerDecision::Forward(frame)) => {
                            cx.shared.can_1_tx_queue.lock(|tx_queue| {
                                tx_queue.push(RawFrame(frame)).unwrap();
                                rtic::pend(Interrupt::USB_HP_CAN_TX);
                            });
                        }
                        Ok(HandlerDecision::Ok) => {}
                        Err(_) => {}
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }
}
