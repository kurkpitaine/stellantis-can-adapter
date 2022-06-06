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

    /// AEE2010 0x128 frame sending task.
    #[task(shared = [can_2_tx_queue])]
    fn send_2010_x128(
        mut cx: send_2010_x128::Context,
        x128_frame_repr: canpsa::aee2010::infodiv::x128::Repr,
    ) {
        let mut x128_buf = [0; canpsa::aee2010::infodiv::x128::FRAME_LEN];
        let mut x128_frame = canpsa::aee2010::infodiv::x128::Frame::new_unchecked(&mut x128_buf);
        x128_frame_repr.emit(&mut x128_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x128).unwrap(),
            Data::new(&x128_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });
    }

    /// AEE2010 0x168 frame sending task.
    #[task(shared = [can_2_tx_queue])]
    fn send_2010_x168(
        mut cx: send_2010_x168::Context,
        x168_frame_repr: canpsa::aee2010::infodiv::x168::Repr,
    ) {
        let mut x168_buf = [0; canpsa::aee2010::infodiv::x168::FRAME_LEN];
        let mut x168_frame = canpsa::aee2010::infodiv::x168::Frame::new_unchecked(&mut x168_buf);
        x168_frame_repr.emit(&mut x168_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x168).unwrap(),
            Data::new(&x168_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });
    }

    /// AEE2010 0x228 frame sending task.
    #[task(shared = [can_2_tx_queue])]
    fn send_2010_x228(
        mut cx: send_2010_x228::Context,
        x228_frame_repr: canpsa::aee2010::infodiv::x228::Repr,
    ) {
        let mut x228_buf = [0; canpsa::aee2010::infodiv::x228::FRAME_LEN];
        let mut x228_frame = canpsa::aee2010::infodiv::x228::Frame::new_unchecked(&mut x228_buf);
        x228_frame_repr.emit(&mut x228_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x228).unwrap(),
            Data::new(&x228_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });
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

    /// AEE2010 0x350 frame sending task.
    #[task(shared = [can_2_tx_queue])]
    fn send_2010_x350(
        mut cx: send_2010_x350::Context,
        x350_frame_repr: canpsa::aee2010::infodiv::x350::Repr,
    ) {
        let mut x350_buf = [0; canpsa::aee2010::infodiv::x350::FRAME_LEN];
        let mut x350_frame = canpsa::aee2010::infodiv::x350::Frame::new_unchecked(&mut x350_buf);
        x350_frame_repr.emit(&mut x350_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x350).unwrap(),
            Data::new(&x350_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });
    }

    /// AEE2010 0x361 frame sending task.
    #[task(shared = [can_2_tx_queue])]
    fn send_2010_x361(
        mut cx: send_2010_x361::Context,
        x361_frame_repr: canpsa::aee2010::infodiv::x361::Repr,
    ) {
        let mut x361_buf = [0; canpsa::aee2010::infodiv::x361::FRAME_LEN];
        let mut x361_frame = canpsa::aee2010::infodiv::x361::Frame::new_unchecked(&mut x361_buf);
        x361_frame_repr.emit(&mut x361_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x361).unwrap(),
            Data::new(&x361_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
        });
    }

    /// AEE2010 0x3e7 frame sending task.
    #[task(shared = [can_2_tx_queue])]
    fn send_2010_x3e7(
        mut cx: send_2010_x3e7::Context,
        x3e7_frame_repr: canpsa::aee2010::infodiv::x3e7::Repr,
    ) {
        let mut x3e7_buf = [0; canpsa::aee2010::infodiv::x3e7::FRAME_LEN];
        let mut x3e7_frame = canpsa::aee2010::infodiv::x3e7::Frame::new_unchecked(&mut x3e7_buf);
        x3e7_frame_repr.emit(&mut x3e7_frame);

        let raw_frame = Frame::new_data(
            StandardId::new(0x3e7).unwrap(),
            Data::new(&x3e7_buf).unwrap(),
        );

        cx.shared.can_2_tx_queue.lock(|tx_queue| {
            tx_queue.push(RawFrame(raw_frame)).unwrap();
            rtic::pend(Interrupt::CAN2_TX);
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
    #[task(binds = USB_LP_CAN_RX0, local = [can_1_rx], shared = [clock])]
    fn can_1_rx0(cx: can_1_rx0::Context) {
        loop {
            match cx.local.can_1_rx.receive() {
                Ok(frame) => {
                    let rx_id = match frame.id() {
                        bxcan::Id::Standard(id) => id.as_raw(),
                        _ => continue,
                    };

                    let rx_data = match frame.data() {
                        Some(data) => data,
                        None => continue,
                    };

                    defmt::info!("CAN AEE2004: received frame [{=u16:#x}]", rx_id);
                    match rx_id {
                        0x128 => {
                            let x128_frame =
                                canpsa::aee2004::conf::x128::Frame::new_checked(rx_data).unwrap();
                            let x128_repr =
                                canpsa::aee2004::conf::x128::Repr::parse(&x128_frame).unwrap();
                            let x128_repr_2010 =
                                canpsa::aee2010::infodiv::x128::Repr::from(&x128_repr);
                            send_2010_x128::spawn(x128_repr_2010).unwrap();
                        }
                        0x168 => {
                            let x168_frame =
                                canpsa::aee2004::conf::x168::Frame::new_checked(rx_data).unwrap();
                            let x168_repr =
                                canpsa::aee2004::conf::x168::Repr::parse(&x168_frame).unwrap();
                            let x168_repr_2010 =
                                canpsa::aee2010::infodiv::x168::Repr::from(&x168_repr);
                            send_2010_x168::spawn(x168_repr_2010).unwrap();
                        }
                        0x1a8 => {
                            let x1a8_frame =
                                canpsa::aee2004::conf::x1a8::Frame::new_checked(rx_data).unwrap();
                            let x1a8_repr =
                                canpsa::aee2004::conf::x1a8::Repr::parse(&x1a8_frame).unwrap();
                            let x228_repr_2010 =
                                canpsa::aee2010::infodiv::x228::Repr::from(&x1a8_repr);
                            send_2010_x228::spawn(x228_repr_2010).unwrap();
                        }
                        0x1d0 => {
                            let x1d0_frame =
                                canpsa::aee2004::conf::x1d0::Frame::new_checked(rx_data).unwrap();
                            let x1d0_repr =
                                canpsa::aee2004::conf::x1d0::Repr::parse(&x1d0_frame).unwrap();
                            let x350_repr_2010 =
                                canpsa::aee2010::infodiv::x350::Repr::from(&x1d0_repr);
                            send_2010_x350::spawn(x350_repr_2010).unwrap();
                        }
                        0x361 => {
                            let x361_frame =
                                canpsa::aee2004::conf::x361::Frame::new_checked(rx_data).unwrap();
                            let x361_repr =
                                canpsa::aee2004::conf::x361::Repr::parse(&x361_frame).unwrap();
                            let x361_repr_2010 =
                                canpsa::aee2010::infodiv::x361::Repr::from(&x361_repr);
                            send_2010_x361::spawn(x361_repr_2010).unwrap();
                        }
                        0x3a7 => {
                            let x3a7_frame =
                                canpsa::aee2004::conf::x3a7::Frame::new_checked(rx_data).unwrap();
                            let x3a7_repr =
                                canpsa::aee2004::conf::x3a7::Repr::parse(&x3a7_frame).unwrap();
                            let x3e7_repr_2010 =
                                canpsa::aee2010::infodiv::x3e7::Repr::from(&x3a7_repr);
                            send_2010_x3e7::spawn(x3e7_repr_2010).unwrap();
                        }
                        _ => {
                            // Forward frame as-is.
                            defmt::info!("CAN AEE2004: forward as-is [{=u16:#x}]", rx_id);
                        }
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }

    /// Receive from AEE2010 Infodiv CAN bus.
    #[task(binds = CAN2_RX0, local = [can_2_rx], shared = [clock])]
    fn can_2_rx0(mut cx: can_2_rx0::Context) {
        loop {
            match cx.local.can_2_rx.receive() {
                Ok(frame) => {
                    let rx_id = match frame.id() {
                        bxcan::Id::Standard(id) => id.as_raw(),
                        _ => continue,
                    };

                    let rx_data = match frame.data() {
                        Some(data) => data,
                        None => continue,
                    };

                    match rx_id {
                        0x39b => {
                            defmt::info!("received 0x39b frame");
                            let x39b_frame =
                                canpsa::aee2010::infodiv::x39b::Frame::new_checked(rx_data)
                                    .unwrap();
                            let x39b_repr =
                                canpsa::aee2010::infodiv::x39b::Repr::parse(&x39b_frame).unwrap();

                            cx.shared.clock.lock(|clk| {
                                let time_epoch = x39b_repr
                                    .utc_datetime
                                    .unix_timestamp()
                                    .saturating_sub(canpsa::UNIX_EPOCH_OFFSET)
                                    as u32;

                                clk.rtc.set_time(time_epoch);
                                clk.clock_format = x39b_repr.clock_format;
                                clk.clock_disp_mode = canpsa::config::DisplayMode::Steady;
                            });
                        }
                        _ => {
                            defmt::info!("Received on CAN AEE2010 unhandled frame[{}]", rx_id);
                        }
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }
}
