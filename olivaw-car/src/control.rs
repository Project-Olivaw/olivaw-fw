//! Routes a decoded [`Control`] command to the task that owns it.

use olivaw_proto::Control;

use crate::shared::{DRIVE_EVENTS, DriveEvent, LIDAR_CTL, UPLINK_CTL};

/// Dispatch one control command. Never blocks: a full drive queue drops
/// the event with a log line rather than stalling the BLE task.
pub fn dispatch(cmd: Control) {
    log::info!("control: {cmd:?}");
    let drive = |ev: DriveEvent| {
        if DRIVE_EVENTS.try_send(ev).is_err() {
            log::warn!("drive event queue full, dropped {ev:?}");
        }
    };
    match cmd {
        Control::EStop => drive(DriveEvent::EStop),
        Control::Clear => drive(DriveEvent::Clear),
        Control::SetMaxDuty(permille) => drive(DriveEvent::SetMaxDuty(permille)),
        Control::LidarOn => LIDAR_CTL.signal(true),
        Control::LidarOff => LIDAR_CTL.signal(false),
        Control::UplinkOn => UPLINK_CTL.signal(true),
        Control::UplinkOff => UPLINK_CTL.signal(false),
        _ => log::warn!("control command not handled: {cmd:?}"),
    }
}
