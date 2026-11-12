// Reminders posted on Dependabot PRs by .github/workflows/dependabot-reminders.yml.
//
// A rule matches when any dependency name matches `deps` or any changed file
// matches `files`. Optional `ecosystems` (e.g. 'cargo', 'github_actions',
// 'submodules') and `updateTypes` ('major', 'minor', 'patch') narrow it further.
// A rule with neither `deps` nor `files` matches on those filters alone.
// Crate patterns use [^/]+ so they don't also match GitHub Actions, whose
// names are owner/repo (e.g. esp-rs/xtensa-toolchain).
// `message` is one Markdown checklist item.
module.exports = [
  {
    id: 'esp-radio-ble-stack',
    deps: [/^esp-radio$/],
    message: `**esp-radio**: lift the matching holds in the \`ignore\` list of \`.github/dependabot.yml\` and move those crates in this PR:
- If this release implements a newer \`bt-hci\`, bump \`bt-hci\` and \`trouble-host\` to match (\`trouble-host\` must use the \`bt-hci\` version \`esp-radio\` implements).
- If it needs a newer \`esp-hal\`, move the whole esp-hal release set together: \`esp-hal\` (exact \`=\` pin), \`esp-rtos\`, \`esp-storage\`, \`esp-alloc\`, \`esp-println\`, \`esp-backtrace\` and \`esp-bootloader-esp-idf\`. Check each one's \`esp-hal\` requirement on crates.io.`,
  },
  {
    id: 'ble-host',
    deps: [/^trouble-host$/, /^bt-hci$/],
    message: `**trouble-host / bt-hci**: \`bt-hci\` must be the version \`esp-radio\` implements (check \`esp-radio\`'s \`bt-hci\` requirement on crates.io). With two \`bt-hci\` versions in \`Cargo.lock\` the ESP board won't compile.`,
  },
  {
    id: 'esp-hal-pin',
    deps: [/^esp-hal$/, /^esp-rtos$/],
    message: `**esp-hal** is pinned exactly (\`=\`) in \`boards/esp/Cargo.toml\` and must satisfy \`esp-radio\`'s and \`esp-rtos\`'s \`esp-hal\` requirement; move them together.`,
  },
  {
    id: 'esp-hardware',
    deps: [/^esp-[^/]+$/, /^trouble-[^/]+$/, /^bt-hci$/, /^embassy-[^/]+$/],
    message: `**ESP targets**: CI only builds the example config's profile. Build the other MCUs (\`cargo xtask build --board esp\` with each \`hardware_profile\`) and flash a board to check BLE pairing, WiFi and CAN still work.`,
  },
  {
    id: 'prost',
    deps: [/^prost(-[^/]+)?$/],
    message: `**prost**: keep \`prost\` and \`prost-build\` on the same version, and check the code generated from \`contracts/\` still builds for every board.`,
  },
  {
    id: 'contracts',
    files: [/^contracts$/],
    message: `**contracts submodule**: check the change follows the contracts' append-only rules, update the firmware for any new fields, and keep the mobile app's pin in step (OTA \`min_app_version\`, see \`docs/releases.md\`).`,
  },
  {
    id: 'release',
    files: [/^(boards|cars|core-interface|hardware-profiles)\/.*Cargo\.toml$/],
    message: `**Release**: this changes a crate manifest, so merging tags a new patch release for every affected platform (\`auto-release.yml\`).`,
  },
  {
    id: 'xtensa-toolchain',
    deps: [/^esp-rs\/xtensa-toolchain$/],
    message: `**xtensa-toolchain** is used in \`ci.yml\` and \`auto-release.yml\`; check both workflows picked up the bump, and keep the dev container's \`espup\` toolchain in step.`,
  },
  {
    id: 'major',
    updateTypes: ['major'],
    message: `**Major update**: read the changelog for breaking changes, and update \`docs/\` if behaviour changes.`,
  },
];
