use bxcan::Frame;

use stm32f1xx_hal::rtc::{Rtc, RtcClkLsi};

use canpsa::*;

#[derive(Debug)]
pub struct RawFrame(pub Frame);

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
    pub rtc: Rtc<RtcClkLsi>,
    /// Format of the time clock, ie: 12/24 hour format.
    pub clock_format: config::ClockFormat,
    /// Clock display mode.
    pub clock_disp_mode: config::DisplayMode,
}

/// Shared vehicle state.
#[derive(Copy, Clone)]
pub struct VehicleState {
    /// Economy mode flag.
    pub economy_mode: bool,
    /// Vehicle main status.
    pub main_status: vehicle::MainStatus,
    /// Generator working flag.
    pub generator_working: bool,
    /// Cockpit steering wheel position,
    /// used to determine which front door is the driver's door.
    pub steering_wheel_position: vehicle::SteeringWheelPosition,
    /// Front left door opene flag.
    pub front_left_door_open: bool,
    /// Front right door open flag.
    pub front_right_door_open: bool,
    /// Rear left door open flag.
    pub rear_left_door_open: bool,
    /// Rear right door open flag.
    pub rear_right_door_open: bool,
    /// Boot open flag.
    pub boot_open: bool,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            economy_mode: false,
            main_status: vehicle::MainStatus::Off,
            generator_working: false,
            steering_wheel_position: vehicle::SteeringWheelPosition::Left,
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
    pub language: config::Language,
    /// Consumption unit used for display.
    pub consumption_unit: config::ConsumptionUnit,
    /// Distance unit used for display.
    pub distance_unit: config::DistanceUnit,
    /// Temperature unit used for display.
    pub temperature_unit: config::TemperatureUnit,
    /// Volume unit used for display.
    pub volume_unit: config::VolumeUnit,
}

impl Default for DisplayConfig {
    fn default() -> Self {
        Self {
            language: config::Language::English,
            consumption_unit: config::ConsumptionUnit::VolumePerDistance,
            distance_unit: config::DistanceUnit::Kilometer,
            temperature_unit: config::TemperatureUnit::Celsius,
            volume_unit: config::VolumeUnit::Liter,
        }
    }
}
