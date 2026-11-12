// Reminders posted on Dependabot PRs by .github/workflows/dependabot-reminders.yml.
//
// A rule matches when any dependency name matches `deps` or any changed file
// matches `files`. Optional `ecosystems` (e.g. 'cargo', 'github_actions',
// 'submodules') and `updateTypes` ('major', 'minor', 'patch') narrow it further.
// A rule with neither `deps` nor `files` matches on those filters alone.
// `message` is one Markdown checklist item.
module.exports = [
  {
    id: 'esp-radio-ble-stack',
    deps: [/^esp-radio$/],
    message: `**esp-radio**: its BLE controller implements one \`bt-hci\` version, and \`trouble-host\` must use that same one. If this release moves to a newer \`bt-hci\`, remove the \`trouble-host\`/\`bt-hci\` \`ignore\` in \`.github/dependabot.yml\` and bump \`bt-hci\` and \`trouble-host\` in \`boards/esp/Cargo.toml\` in this PR.`,
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
    deps: [/^esp-/, /^trouble-/, /^bt-hci$/, /^embassy-/],
    message: `**ESP targets**: CI only builds the example config's profile. Build the other MCUs (\`cargo xtask build --board esp\` with each \`hardware_profile\`) and flash a board to check BLE pairing, WiFi and CAN still work.`,
  },
  {
    id: 'prost',
    deps: [/^prost/],
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
