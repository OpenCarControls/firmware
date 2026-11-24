//! `RestartCommand` on the PC board: the process replaces itself with a fresh copy of
//! the same binary and arguments, so it boots again as the ESP does after a reset.

/// Writes what is pending to the store files, then restarts the process. Only returns if
/// the new process couldn't be started.
pub(crate) fn flush_and_restart() {
    crate::can_approval::flush_pending_approval();
    crate::phone_validations::flush_pending_validations();
    let exe = match std::env::current_exe() {
        Ok(exe) => exe,
        Err(e) => {
            log::error!(
                "SYSTEM: restart failed, can't find the running binary: {}",
                e
            );
            return;
        }
    };
    let mut command = std::process::Command::new(exe);
    command.args(std::env::args_os().skip(1));
    log::warn!("SYSTEM: restarting");
    // Every file and socket std opens is close-on-exec, so the new process can bind the
    // same ports.
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt as _;
        let e = command.exec();
        log::error!("SYSTEM: restart failed: {}", e);
    }
    #[cfg(not(unix))]
    match command.spawn() {
        Ok(_) => std::process::exit(0),
        Err(e) => log::error!("SYSTEM: restart failed: {}", e),
    }
}
