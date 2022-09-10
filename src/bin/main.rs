#![no_main]
#![no_std]

use stellantis_can_adapter as _; // global logger + panicking-behavior + memory layout

use bxcan::{filter::Mask32, Data, Fifo, Frame, Interrupts, Rx0, StandardId, Tx};
use heapless::binary_heap::{BinaryHeap, Max};

use stm32f1xx_hal::{
    can::Can,
    pac::{Interrupt, CAN1, CAN2, TIM2},
    prelude::*,
    rtc::Rtc,
    timer::monotonic::MonoTimer,
};

use canpsa::config::UserProfile;

mod handlers;
use handlers::*;

mod utils;
use utils::*;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[local]
    struct Local {
        can_1_tx: Tx<Can<CAN1>>,
        can_1_rx: Rx0<Can<CAN1>>,
        can_2_tx: Tx<Can<CAN2>>,
        can_2_rx: Rx0<Can<CAN2>>,
        x0e6_2010_checksum_counter: u8,
    }
    #[shared]
    struct Shared {
        clock: ClockState,
        can_1_tx_queue: BinaryHeap<RawFrame, Max, 16>,
        can_2_tx_queue: BinaryHeap<RawFrame, Max, 16>,
        vehicle_state: VehicleState,
        display_config: DisplayConfig,
        timeout_handle: Option<no_data_from_vsm_timeout::SpawnHandle>,
        send_2004_0x15b_task_handle: Option<send_2004_0x15b::SpawnHandle>,
        send_2004_0x228_task_handle: Option<send_2004_0x228::SpawnHandle>,
        send_2004_0x376_0x3f6_and_2010_0x276_task_handle:
            Option<send_2004_0x376_0x3f6_and_2010_0x276::SpawnHandle>,
        send_2004_0x525_task_handle:
            Option<send_2004_0x525::SpawnHandle>,
        send_2010_0x122_task_handle: Option<send_2010_0x122::SpawnHandle>,
        send_2010_0x227_task_handle: Option<send_2010_0x227::SpawnHandle>,
        send_2010_0x236_task_handle: Option<send_2010_0x236::SpawnHandle>,
        vehicle_network_state: canpsa::vehicle::NetworkState,
        x15b_2004_repr: Option<canpsa::aee2004::conf::x15b::Repr>,
        x260_2004_repr: Option<canpsa::aee2004::conf::x260::Repr>,
        x227_2010_repr: Option<canpsa::aee2010::infodiv::x227::Repr>,
        debounce_2010_0x1a9: bool,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MyMono = MonoTimer<TIM2, 1000>;

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
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

        // Set SLEEPONEXIT bit to lower power usage without putting MCU in standby mode.
        cx.core.SCB.set_sleeponexit();

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
                timeout_handle: Option::None,
                send_2004_0x15b_task_handle: Option::None,
                send_2004_0x228_task_handle: Option::None,
                send_2004_0x376_0x3f6_and_2010_0x276_task_handle: Option::None,
                send_2004_0x525_task_handle: Option::None,
                send_2010_0x122_task_handle: Option::None,
                send_2010_0x227_task_handle: Option::None,
                send_2010_0x236_task_handle: Option::None,
                vehicle_network_state: canpsa::vehicle::NetworkState::Sleep,
                x15b_2004_repr: Option::None,
                x260_2004_repr: Option::None,
                x227_2010_repr: Option::None,
                debounce_2010_0x1a9: false,
            },
            Local {
                can_1_tx,
                can_1_rx,
                can_2_tx,
                can_2_rx,
                x0e6_2010_checksum_counter: 0,
            },
            init::Monotonics(mono),
        )
    }

    #[idle()]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Now Wait For Interrupt is used instead of a busy-wait loop
            // to allow MCU to sleep between interrupts
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            rtic::export::wfi()
        }
    }

    /// AEE2004 0x15b frame sending task.
    #[task(shared = [can_1_tx_queue, send_2004_0x15b_task_handle, x15b_2004_repr])]
    fn send_2004_0x15b(mut cx: send_2004_0x15b::Context) {
        // Retrieve 0x15b frame content
        let x15b_frame_repr = cx.shared.x15b_2004_repr.lock(|repr| *repr);

        // Send only if we have content. Else, let task die.
        x15b_frame_repr.map_or(
            {
                cx.shared.send_2004_0x15b_task_handle.lock(|handle| {
                    *handle = None;
                });
            },
            |frame_repr| {
                let mut x15b_buf = [0; canpsa::aee2004::conf::x15b::FRAME_LEN];
                let mut x15b_frame =
                    canpsa::aee2004::conf::x15b::Frame::new_unchecked(&mut x15b_buf);
                frame_repr.emit(&mut x15b_frame);

                let raw_frame = Frame::new_data(
                    StandardId::new(canpsa::aee2004::conf::x15b::FRAME_ID).unwrap(),
                    Data::new(&x15b_buf).unwrap(),
                );

                cx.shared.can_1_tx_queue.lock(|tx_queue| {
                    tx_queue.push(RawFrame(raw_frame)).ok();
                    rtic::pend(Interrupt::USB_HP_CAN_TX);
                });

                let new_handle = send_2004_0x15b::spawn_after(500.millis()).unwrap();
                cx.shared.send_2004_0x15b_task_handle.lock(|handle| {
                    *handle = Some(new_handle);
                });
            },
        );
    }

    /// AEE2004 0x228 frame sending task.
    #[task(shared = [can_1_tx_queue, clock, send_2004_0x228_task_handle])]
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
            StandardId::new(canpsa::aee2004::conf::x228::FRAME_ID).unwrap(),
            Data::new(&x228_buf).unwrap(),
        );

        cx.shared.can_1_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).ok();
            rtic::pend(Interrupt::USB_HP_CAN_TX);
        });

        let new_handle = send_2004_0x228::spawn_after(60.secs()).unwrap();
        cx.shared.send_2004_0x228_task_handle.lock(|handle| {
            *handle = Some(new_handle);
        });
    }

    /// AEE2004 0x5e5 frame sending task.
    #[task(shared = [can_1_tx_queue])]
    fn send_2004_0x5e5(mut cx: send_2004_0x5e5::Context) {
        let x5e5_buf = [0x25, 0x1c, 0x05, 0x09, 0x48, 0x01, 0x20, 0x29];

        let raw_frame = Frame::new_data(
            StandardId::new(0x5e5).unwrap(),
            Data::new(&x5e5_buf).unwrap(),
        );

        cx.shared.can_1_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).ok();
            rtic::pend(Interrupt::USB_HP_CAN_TX);
        });
    }

    /// AEE2004 0x525 frame sending task.
    #[task(shared = [can_1_tx_queue, send_2004_0x525_task_handle])]
    fn send_2004_0x525(mut cx: send_2004_0x525::Context) {
        let x525_buf = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

        let raw_frame = Frame::new_data(
            StandardId::new(0x525).unwrap(),
            Data::new(&x525_buf).unwrap(),
        );

        cx.shared.can_1_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).ok();
            rtic::pend(Interrupt::USB_HP_CAN_TX);
        });

        let new_handle = send_2004_0x525::spawn_after(1.secs()).unwrap();
        cx.shared.send_2004_0x525_task_handle.lock(|handle| {
            *handle = Some(new_handle);
        });
    }

    /// AEE2010 0x122 frame sending task.
    #[task(shared = [can_2_tx_queue, send_2010_0x122_task_handle])]
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
            StandardId::new(canpsa::aee2010::infodiv::x122::FRAME_ID).unwrap(),
            Data::new(&x122_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).ok();
            rtic::pend(Interrupt::CAN2_TX);
        });

        let new_handle = send_2010_0x122::spawn_after(200.millis()).unwrap();
        cx.shared.send_2010_0x122_task_handle.lock(|handle| {
            *handle = Some(new_handle);
        });
    }

    /// AEE2004 0x376 + 0x3f6 and AEE2010 0x276 frames sending task.
    #[task(shared = [can_1_tx_queue, can_2_tx_queue, clock, display_config, send_2004_0x376_0x3f6_and_2010_0x276_task_handle])]
    fn send_2004_0x376_0x3f6_and_2010_0x276(mut cx: send_2004_0x376_0x3f6_and_2010_0x276::Context) {
        // Retrieve clock data.
        let clock_data = cx.shared.clock.lock(|clk| {
            (
                clk.rtc.current_time(),
                clk.clock_format,
                clk.clock_disp_mode,
            )
        });

        // Retrieve display configuration.
        let display_config = cx
            .shared
            .display_config
            .lock(|display_config| *display_config);

        // Calculate time from the value returned by the RTC.
        let offset_datetime = time::OffsetDateTime::from_unix_timestamp(
            clock_data.0 as i64 + canpsa::UNIX_EPOCH_OFFSET,
        )
        .unwrap();

        /*                     */
        /* AEE2004 x376 frame. */
        /*                     */

        // Fill x376 frame Repr.
        let x376_frame_repr = canpsa::aee2004::conf::x376::Repr {
            clock_disp_mode: clock_data.2,
            utc_datetime: offset_datetime,
        };

        // Serialize x376 frame Repr into buffer.
        let mut x376_buf = [0; canpsa::aee2004::conf::x376::FRAME_LEN];
        let mut x376_frame = canpsa::aee2004::conf::x376::Frame::new_unchecked(&mut x376_buf);
        x376_frame_repr.emit(&mut x376_frame);

        // Transform serialized x376 frame into a Can frame.
        let raw_x376_frame = Frame::new_data(
            StandardId::new(canpsa::aee2004::conf::x376::FRAME_ID).unwrap(),
            Data::new(&x376_buf).unwrap(),
        );

        /*                     */
        /* AEE2004 x3f6 frame. */
        /*                     */

        // Fill x3f6 frame Repr.
        let x3f6_frame_repr = canpsa::aee2004::conf::x3f6::Repr {
            running_duration: time::Duration::new(0, 0),
            distance_unit: display_config.distance_unit,
            volume_unit: display_config.volume_unit,
            consumption_unit: display_config.consumption_unit,
            pressure_unit: canpsa::config::PressureUnit::Bar,
            display_charset: canpsa::config::DisplayCharset::ASCII,
            temperature_unit: display_config.temperature_unit,
            display_mode: canpsa::config::DisplayColorMode::Negative,
            clock_format: clock_data.1,
            language: display_config.language,
        };

        // Serialize x3f6 frame Repr into buffer.
        let mut x3f6_buf = [0; canpsa::aee2004::conf::x3f6::FRAME_LEN];
        let mut x3f6_frame = canpsa::aee2004::conf::x3f6::Frame::new_unchecked(&mut x3f6_buf);
        x3f6_frame_repr.emit(&mut x3f6_frame);

        // Transform serialized x3f6 frame into a Can frame.
        let raw_x3f6_frame = Frame::new_data(
            StandardId::new(canpsa::aee2004::conf::x3f6::FRAME_ID).unwrap(),
            Data::new(&x3f6_buf).unwrap(),
        );

        /*                     */
        /* AEE2010 x276 frame. */
        /*                     */

        // Fill x276 frame Repr.
        let x276_frame_repr = canpsa::aee2010::infodiv::x276::Repr {
            clock_format: clock_data.1,
            clock_disp_mode: clock_data.2,
            utc_datetime: offset_datetime,
            adblue_autonomy: 0xfffe, // Unavailable
            adblue_autonomy_display_request: false,
        };

        // Serialize x276 frame Repr into buffer.
        let mut x276_buf = [0; canpsa::aee2010::infodiv::x276::FRAME_LEN];
        let mut x276_frame = canpsa::aee2010::infodiv::x276::Frame::new_unchecked(&mut x276_buf);
        x276_frame_repr.emit(&mut x276_frame);

        // Transform serialized x276 frame into a Can frame.
        let raw_x276_frame = Frame::new_data(
            StandardId::new(canpsa::aee2010::infodiv::x276::FRAME_ID).unwrap(),
            Data::new(&x276_buf).unwrap(),
        );

        // Put AEE2004 x376 and x3f6 frames into the CAN 1 Tx queue.
        cx.shared.can_1_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_x376_frame)).ok();
            tx_queue.push(RawFrame(raw_x3f6_frame)).ok();
            rtic::pend(Interrupt::USB_HP_CAN_TX);
        });

        // Put AEE2010 x276 frame into the CAN 2 Tx queue.
        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_x276_frame)).ok();
            rtic::pend(Interrupt::CAN2_TX);
        });

        let new_handle = send_2004_0x376_0x3f6_and_2010_0x276::spawn_after(1.secs()).unwrap();
        cx.shared
            .send_2004_0x376_0x3f6_and_2010_0x276_task_handle
            .lock(|handle| {
                *handle = Some(new_handle);
            });
    }

    /// AEE2010 0x227 frame sending task.
    #[task(shared = [can_2_tx_queue, send_2010_0x227_task_handle, x15b_2004_repr, x227_2010_repr])]
    fn send_2010_0x227(mut cx: send_2010_0x227::Context) {
        // Retrieve 0x227 frame content
        let x227_frame_repr = cx.shared.x227_2010_repr.lock(|repr| *repr);

        // Send only if we have content. Else, let task die.
        x227_frame_repr.map_or(
            {
                cx.shared.send_2010_0x227_task_handle.lock(|handle| {
                    *handle = None;
                });
            },
            |mut frame_repr| {
                // Add parking sensors LED state
                frame_repr.parking_sensors_led_state =
                    cx.shared
                        .x15b_2004_repr
                        .lock(|x15b_repr_opt| match x15b_repr_opt {
                            Some(x15b_repr) if x15b_repr.park_sensors_status == 0 => {
                                canpsa::vehicle::PushButtonLedState::Steady
                            }
                            _ => canpsa::vehicle::PushButtonLedState::Off,
                        });

                let mut x227_buf = [0; canpsa::aee2010::infodiv::x227::FRAME_LEN];
                let mut x227_frame =
                    canpsa::aee2010::infodiv::x227::Frame::new_unchecked(&mut x227_buf);
                frame_repr.emit(&mut x227_frame);

                let raw_frame = Frame::new_data(
                    StandardId::new(canpsa::aee2010::infodiv::x227::FRAME_ID).unwrap(),
                    Data::new(&x227_buf).unwrap(),
                );

                cx.shared.can_2_tx_queue.lock(|tx_queue| {
                    tx_queue.push(RawFrame(raw_frame)).ok();
                    rtic::pend(Interrupt::CAN2_TX);
                });

                let new_handle = send_2010_0x227::spawn_after(500.millis()).unwrap();
                cx.shared.send_2010_0x227_task_handle.lock(|handle| {
                    *handle = Some(new_handle);
                });
            },
        );
    }

    /// AEE2010 0x236 frame sending task.
    #[task(shared = [can_2_tx_queue, vehicle_state, send_2010_0x236_task_handle])]
    fn send_2010_0x236(mut cx: send_2010_0x236::Context) {
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
            StandardId::new(canpsa::aee2010::infodiv::x236::FRAME_ID).unwrap(),
            Data::new(&x236_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).ok();
            rtic::pend(Interrupt::CAN2_TX);
        });

        let new_handle = send_2010_0x236::spawn_after(500.millis()).unwrap();
        cx.shared.send_2010_0x236_task_handle.lock(|handle| {
            *handle = Some(new_handle);
        });
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
                            tx_queue.push(RawFrame(pending_frame.clone())).ok();
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
                            tx_queue.push(RawFrame(pending_frame.clone())).ok();
                        }
                    },
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => unreachable!(),
                }
            }
        });
    }

    /// Receive from AEE2004 Conf CAN bus.
    #[task(binds = USB_LP_CAN_RX0, local = [can_1_rx, x0e6_2010_checksum_counter], shared = [can_2_tx_queue, vehicle_state, display_config, timeout_handle, vehicle_network_state, send_2010_0x227_task_handle, x15b_2004_repr, x260_2004_repr, x227_2010_repr])]
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
                            // A standard cycle is : WakeUp -> Normal -> GoingToSleep -> Sleep
                            // The rule is : talk when NetworkState is Normal, else be silent.
                            match repr.network_state {
                                canpsa::vehicle::NetworkState::Normal => {
                                    cx.shared.timeout_handle.lock(|handle_opt| {
                                        // (Re)schedule timeout
                                        handle_opt.take().map(|handle| handle.cancel().ok());
                                        *handle_opt =
                                            no_data_from_vsm_timeout::spawn_after(300.millis())
                                                .ok();
                                    });

                                    // Shoot frames
                                    cx.shared.vehicle_network_state.lock(|net_state| {
                                        if *net_state != repr.network_state {
                                            send_2004_0x228::spawn().ok();
                                            send_2004_0x376_0x3f6_and_2010_0x276::spawn().ok();
                                            send_2004_0x525::spawn().ok();
                                            send_2004_0x5e5::spawn().ok();
                                            send_2010_0x122::spawn().ok();
                                            send_2010_0x236::spawn().ok();
                                        }

                                        // Update internal network state with new value.
                                        *net_state = repr.network_state;
                                    });
                                }
                                _ => {
                                    cx.shared.timeout_handle.lock(|handle_opt| {
                                        // Cancel timeout task.
                                        handle_opt.take().map(|handle| handle.cancel().ok());
                                    });

                                    cx.shared.vehicle_network_state.lock(|net_state| {
                                        // Update internal network state with new value.
                                        *net_state = repr.network_state;
                                    });

                                    // Stop periodic frames sending.
                                    stop_periodic_frames::spawn().ok();
                                }
                            };

                            cx.shared.vehicle_state.lock(|state| {
                                state.economy_mode = repr.economy_mode_enabled;
                            });

                            Ok(HandlerDecision::Forward(recv_frame))
                        }),
                        0x0e6 => parse_2004_x0e6(rx_data).and_then(|repr| {
                            let fwd_repr = canpsa::aee2010::infodiv::x0e6::Repr::from(&repr);
                            encode_2010_x0e6(&fwd_repr, cx.local.x0e6_2010_checksum_counter)
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
                        0x227 => parse_2004_x227(rx_data).and_then(|repr| {
                            let fwd_repr = canpsa::aee2010::infodiv::x227::Repr::from(&repr);
                            cx.shared
                                .x227_2010_repr
                                .lock(|x227_repr_opt| *x227_repr_opt = Some(fwd_repr));

                            // Check if we must start x227 task
                            let should_start = cx
                                .shared
                                .send_2010_0x227_task_handle
                                .lock(|handle_opt| handle_opt.is_none());

                            if should_start {
                                restart_2010_x227_task::spawn().ok();
                            }

                            Ok(HandlerDecision::Ok)

                            /* fwd_repr.parking_sensors_led_state = cx.shared.x15b_2004_repr.lock(
                                |x15b_repr_opt| match x15b_repr_opt {
                                    Some(x15b_repr) if x15b_repr.park_sensors_status == 0 => {
                                        canpsa::vehicle::PushButtonLedState::Steady
                                    }
                                    _ => canpsa::vehicle::PushButtonLedState::Off,
                                },
                            );

                            encode_2010_x227(&fwd_repr) */
                        }),
                        0x260 => parse_2004_x260(rx_data).and_then(|repr| {
                            // Here, the car sends profile 1 then profile 2 then profile 3 frames in cycle.
                            // We should forward only the profile 1 frame, not sending 2 and 3 to the AEE2010 side.
                            // We don't send anything before we got at least one x260 for profile 1.

                            match repr.profile_number {
                                UserProfile::Profile1 => {
                                    let x260 = Option::Some(repr);
                                    cx.shared.x260_2004_repr.lock(|x260_repr| {
                                        *x260_repr = x260;
                                    });

                                    // Sync internal parking sensors state with VSM's.
                                    cx.shared.x15b_2004_repr.lock(|x15b_repr_opt| {
                                        if let Some(x15b_repr) = x15b_repr_opt {
                                            x15b_repr.park_sensors_status =
                                                repr.park_sensors_status;
                                        }
                                    });
                                    x260
                                }
                                _ => cx.shared.x260_2004_repr.lock(|x260_repr| *x260_repr),
                            }
                            .map_or(
                                Ok(HandlerDecision::Ok),
                                |in_repr| {
                                    let mut fwd_repr =
                                        canpsa::aee2010::infodiv::x260::Repr::from(&in_repr);

                                    // It seems like parameters validity flag is never set on AEE2004.
                                    // We need to force it on AEE2010 to display the settings on the NAC.
                                    fwd_repr.parameters_validity = true;

                                    cx.shared.display_config.lock(|config| {
                                        fwd_repr.consumption_unit = config.consumption_unit;
                                        fwd_repr.distance_unit = config.distance_unit;
                                        fwd_repr.language = config.language;
                                        fwd_repr.temperature_unit = config.temperature_unit;
                                        fwd_repr.volume_unit = config.volume_unit;
                                    });

                                    encode_2010_x260(&fwd_repr)
                                },
                            )
                        }),
                        0x336 => {
                            let data = [0x56, 0x46, 0x37];
                            let frame = Frame::new_data(
                                StandardId::new(0x336).unwrap(),
                                Data::new(&data).unwrap(),
                            );
                            Ok(HandlerDecision::Forward(frame))
                        }
                        0x3b6 => {
                            let data = [0x30, 0x50, 0x48, 0x4E, 0x59, 0x48];
                            let frame = Frame::new_data(
                                StandardId::new(0x3b6).unwrap(),
                                Data::new(&data).unwrap(),
                            );
                            Ok(HandlerDecision::Forward(frame))
                        }
                        0x2b6 => {
                            let data = [0x48, 0x45, 0x35, 0x34, 0x34, 0x36, 0x32, 0x38];
                            let frame = Frame::new_data(
                                StandardId::new(0x2b6).unwrap(),
                                Data::new(&data).unwrap(),
                            );
                            Ok(HandlerDecision::Forward(frame))
                        }
                        0x128 => handle_2004_x128(rx_data),
                        0x168 => handle_2004_x168(rx_data),
                        0x1a8 => handle_2004_x1a8(rx_data),
                        0x1d0 => handle_2004_x1d0(rx_data),
                        0x1e1 => handle_2004_x1e1(rx_data),
                        0x261 => handle_2004_x261(rx_data),
                        0x2a1 => handle_2004_x2a1(rx_data),
                        0x361 => handle_2004_x361(rx_data),
                        0x3a7 => handle_2004_x3a7(rx_data),
                        0x228 => Ok(HandlerDecision::Ok), // Filter received 0x228 if this happens
                        _ => {
                            // Forward frame as-is.
                            Ok(HandlerDecision::Forward(recv_frame))
                        }
                    };

                    match result {
                        Ok(HandlerDecision::Forward(frame)) => {
                            cx.shared.can_2_tx_queue.lock(|tx_queue| {
                                tx_queue.push(RawFrame(frame)).ok();
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
    #[task(binds = CAN2_RX0, local = [can_2_rx], shared = [can_1_tx_queue, clock, display_config, x15b_2004_repr, x260_2004_repr, debounce_2010_0x1a9])]
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
                            // Faulty frames with this value set to true are sent at startup.
                            // We need to filter them to avoid wrong settings in the VSM.
                            match repr.auto_mirrors_folding_inhibit {
                                true => Ok(HandlerDecision::Ok),
                                false => {
                                    // #15b is sent a first time when the NAC receives the #260 and #361 frames.
                                    // Then, it is only sent when there is a modification in the vehicle settings.
                                    // In AEE2004, this frame must be sent every 500ms to the VSM.
                                    // The process here is:
                                    // - On reception on AEE2010 side of #15b frame, cancel the AEE2004 #15b periodic task,
                                    //   store new #15b representation and start a new AEE2004 periodic task.

                                    let mut fwd_repr =
                                        canpsa::aee2004::conf::x15b::Repr::from(&repr);

                                    // It seems like parameters validity flag is never set on AEE2004.
                                    // We need to force it off AEE2004 to be able to modify the VSM parameters.
                                    fwd_repr.parameters_validity = false;

                                    // Save display config parameters.
                                    if repr.units_language_parameters_validity {
                                        cx.shared.display_config.lock(|config| {
                                            config.consumption_unit = repr.consumption_unit;
                                            config.distance_unit = repr.distance_unit;
                                            config.language = repr.language;
                                            config.temperature_unit = repr.temperature_unit;
                                            config.volume_unit = repr.volume_unit;
                                        });
                                    }

                                    // Save x15b frame for periodic sending.
                                    cx.shared.x15b_2004_repr.lock(|repr_opt| {
                                        *repr_opt = Some(fwd_repr);
                                    });

                                    // Apply temporary the x15b content into the x260 frame.
                                    // This is needed because the VSM does not send immediately the
                                    // status of the x15b command response in AEE2004.
                                    cx.shared.x260_2004_repr.lock(|repr_opt| {
                                        if repr_opt.is_some() {
                                            *repr_opt = Some(
                                                canpsa::aee2004::conf::x260::Repr::from(&fwd_repr),
                                            );
                                        }
                                    });

                                    // (Re)start AEE2004 x15b sending task.
                                    restart_2004_x15b_task::spawn().ok();

                                    Ok(HandlerDecision::Ok)
                                }
                            }
                        }),
                        0x1a9 => parse_2010_x1a9(rx_data).and_then(|repr| {
                            // Update internal vehicle state for parking sensors if not debouncing frame.
                            cx.shared.debounce_2010_0x1a9.lock(|debouncing| {
                                if repr.parking_sensors_button_state && !*debouncing {
                                    cx.shared.x15b_2004_repr.lock(|x15b_repr_opt| {
                                        match x15b_repr_opt {
                                            Some(x15b_repr)
                                                if x15b_repr.park_sensors_status > 0 =>
                                            {
                                                x15b_repr.park_sensors_status = 0;
                                            }
                                            Some(x15b_repr) => {
                                                x15b_repr.park_sensors_status = 3;
                                            }
                                            None => {}
                                        }
                                    });

                                    // Apply also to the local x260 frame, as the NAC is mirroring it.
                                    cx.shared.x260_2004_repr.lock(|x260_repr_opt| {
                                        match x260_repr_opt {
                                            Some(x260_repr)
                                                if x260_repr.park_sensors_status > 0 =>
                                            {
                                                x260_repr.park_sensors_status = 0;
                                            }
                                            Some(x260_repr) => {
                                                x260_repr.park_sensors_status = 3;
                                            }
                                            None => {}
                                        }
                                    });

                                    // When sending x1a9 upon a driver request (action on a switch on NAC screen):
                                    //  * An x1a9 containing the switch status is immediately sent
                                    //  * x1a9 period is set to 30ms during max 100ms
                                    //  * waits 35ms
                                    //  * resume "standard" x1a9 frame sending at a 200ms period
                                    // All this stuff is to simulate an action on a push button.
                                    // Debounce the 0x1a9 sending to avoid doing this again.
                                    *debouncing = true;

                                    debounce_2010_0x1a9_timeout::spawn_after(100.millis()).unwrap();

                                    // Restart x15b task to notify the VSM.
                                    restart_2004_x15b_task::spawn().ok();

                                    // Restart x227 task to notify the NAC parameters have been accepted.
                                    restart_2010_x227_task::spawn().ok();
                                }
                            });

                            let fwd_repr = canpsa::aee2004::conf::x167::Repr {
                                mfd_trip_computer_page: canpsa::mfd::TripComputerPage::Nothing,
                                maintenance_reset_request: true, // Inverted, or PSA doc is wrong.
                                emergency_call_in_progress: false,
                                fault_recall_request: repr.fault_check_request,
                                trip_computer_secondary_trip_reset_request: repr
                                    .trip_computer_secondary_trip_reset_request,
                                trip_computer_primary_trip_reset_request: repr
                                    .trip_computer_primary_trip_reset_request,
                                pre_conditioning_time: 0,
                                telematics_enabled: repr.telematics_enabled,
                                black_panel_enabled: repr.black_panel_enabled,
                                indirect_under_inflation_reset_request: repr
                                    .indirect_under_inflation_button_state,
                                pre_conditioning_request: false,
                                total_trip_distance: 0xffff,
                                interactive_message: 0,
                                stop_and_start_button_state: repr.stop_start_button_state,
                                lane_centering_button_state: repr.lane_centering_button_state,
                                parking_sensors_button_state: repr.parking_sensors_button_state, // No action in reality
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
                            Ok(HandlerDecision::Forward(recv_frame))
                        }
                    };

                    match result {
                        Ok(HandlerDecision::Forward(frame)) => {
                            cx.shared.can_1_tx_queue.lock(|tx_queue| {
                                tx_queue.push(RawFrame(frame)).ok();
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

    #[task(shared = [send_2004_0x15b_task_handle, send_2004_0x228_task_handle, send_2004_0x525_task_handle, send_2004_0x376_0x3f6_and_2010_0x276_task_handle, send_2010_0x122_task_handle, send_2010_0x227_task_handle, send_2010_0x236_task_handle])]
    fn stop_periodic_frames(mut cx: stop_periodic_frames::Context) {
        cx.shared.send_2004_0x15b_task_handle.lock(|handle_opt| {
            handle_opt.take().map(|handle| handle.cancel().ok());
        });

        cx.shared.send_2004_0x228_task_handle.lock(|handle_opt| {
            handle_opt.take().map(|handle| handle.cancel().ok());
        });

        cx.shared.send_2004_0x525_task_handle.lock(|handle_opt| {
            handle_opt.take().map(|handle| handle.cancel().ok());
        });

        cx.shared
            .send_2004_0x376_0x3f6_and_2010_0x276_task_handle
            .lock(|handle_opt| {
                handle_opt.take().map(|handle| handle.cancel().ok());
            });

        cx.shared.send_2010_0x122_task_handle.lock(|handle_opt| {
            handle_opt.take().map(|handle| handle.cancel().ok());
        });

        cx.shared.send_2010_0x227_task_handle.lock(|handle_opt| {
            handle_opt.take().map(|handle| handle.cancel().ok());
        });

        cx.shared.send_2010_0x236_task_handle.lock(|handle_opt| {
            handle_opt.take().map(|handle| handle.cancel().ok());
        });
    }

    #[task(shared = [vehicle_network_state])]
    fn no_data_from_vsm_timeout(mut cx: no_data_from_vsm_timeout::Context) {
        cx.shared.vehicle_network_state.lock(|net_state| {
            *net_state = canpsa::vehicle::NetworkState::Sleep;
        });

        stop_periodic_frames::spawn().ok();
    }

    #[task(shared = [vehicle_network_state, send_2004_0x15b_task_handle])]
    fn restart_2004_x15b_task(mut cx: restart_2004_x15b_task::Context) {
        cx.shared.vehicle_network_state.lock(|net_state| {
            if *net_state == canpsa::vehicle::NetworkState::Normal {
                cx.shared.send_2004_0x15b_task_handle.lock(|handle_opt| {
                    handle_opt.take().map(|handle| handle.cancel().ok());
                });

                send_2004_0x15b::spawn().ok();
            }
        });
    }

    #[task(shared = [vehicle_network_state, send_2010_0x227_task_handle])]
    fn restart_2010_x227_task(mut cx: restart_2010_x227_task::Context) {
        cx.shared.vehicle_network_state.lock(|net_state| {
            if *net_state == canpsa::vehicle::NetworkState::Normal {
                cx.shared.send_2010_0x227_task_handle.lock(|handle_opt| {
                    handle_opt.take().map(|handle| handle.cancel().ok());
                });

                send_2010_0x227::spawn().ok();
            }
        });
    }

    #[task(shared = [debounce_2010_0x1a9])]
    fn debounce_2010_0x1a9_timeout(mut cx: debounce_2010_0x1a9_timeout::Context) {
        cx.shared.debounce_2010_0x1a9.lock(|debouncing| {
            *debouncing = false;
        });
    }
}
