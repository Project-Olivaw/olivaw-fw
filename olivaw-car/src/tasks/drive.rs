//! 50 Hz drive loop: watchdog → E-stop → inhibit → governor → slew → L298N.

use embassy_time::{Duration, Ticker};
use olivaw_board_esp32::CarMotors;
use olivaw_drive::{DriveController, DriveOutput, Motors as _};
use olivaw_proto::Flags;

use crate::config;
use crate::shared::{DRIVE_CMD, DRIVE_EVENTS, DriveEvent, now_ms, with_state};

/// Owns the motors for the life of the firmware.
#[embassy_executor::task]
pub async fn run(mut motors: CarMotors) {
    let mut ctl = DriveController::new(config::DRIVE);
    let mut ticker = Ticker::every(Duration::from_millis(config::DRIVE_TICK_MS));
    log::info!("drive: {:?}", config::DRIVE);

    loop {
        ticker.next().await;
        let now = now_ms();

        if let Some(cmd) = DRIVE_CMD.try_take() {
            ctl.feed(cmd, now);
        }
        while let Ok(event) = DRIVE_EVENTS.try_receive() {
            match event {
                DriveEvent::EStop => ctl.estop(),
                DriveEvent::Clear => ctl.clear(),
                DriveEvent::SetMaxDuty(permille) => ctl.set_max_duty(permille),
                DriveEvent::LinkLost => ctl.link_lost(),
                DriveEvent::Inhibit(on) => ctl.set_inhibited(on),
            }
        }

        let out = ctl.tick(now);
        let applied = match out {
            DriveOutput::Brake => motors.brake(),
            DriveOutput::Drive(cmd) => motors.set(cmd),
        };
        if let Err(e) = applied {
            log::error!("motor driver: {e:?}");
        }

        let cmd = out.command();
        with_state(|s| {
            s.left = cmd.left;
            s.right = cmd.right;
            s.max_duty = ctl.max_duty();
            s.flags.set(Flags::E_STOP, ctl.is_estopped());
            s.flags.set(Flags::WATCHDOG_STALE, ctl.is_stale());
        });
    }
}
