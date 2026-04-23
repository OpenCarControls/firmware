use core_interface::{
    add_paired_phone, clear_paired_phones, is_phone_paired, list_paired_phones, paired_phone_count,
    remove_paired_phone, set_ble_max_bonded_phones,
};

fn id(bytes: &[u8]) -> Vec<u8> {
    bytes.to_vec()
}

#[test]
fn add_and_list_paired_phones() {
    embassy_futures::block_on(clear_paired_phones());
    set_ble_max_bonded_phones(8);

    assert!(embassy_futures::block_on(add_paired_phone(&id(&[1, 2, 3]))));
    assert!(embassy_futures::block_on(add_paired_phone(&id(&[4, 5, 6]))));

    let phones = embassy_futures::block_on(list_paired_phones());
    assert_eq!(phones.len(), 2);
    assert!(phones.iter().any(|p| p.as_slice() == [1, 2, 3]));
    assert!(phones.iter().any(|p| p.as_slice() == [4, 5, 6]));
}

#[test]
fn add_rejects_empty_device_id() {
    embassy_futures::block_on(clear_paired_phones());
    assert!(!embassy_futures::block_on(add_paired_phone(&[])));
    assert_eq!(embassy_futures::block_on(paired_phone_count()), 0);
}

#[test]
fn add_respects_max_bond_limit() {
    embassy_futures::block_on(clear_paired_phones());
    set_ble_max_bonded_phones(1);

    assert!(embassy_futures::block_on(add_paired_phone(&id(&[0xAA]))));
    assert!(!embassy_futures::block_on(add_paired_phone(&id(&[0xBB]))));
    assert_eq!(embassy_futures::block_on(paired_phone_count()), 1);

    set_ble_max_bonded_phones(8);
}

#[test]
fn add_duplicate_does_not_increase_count() {
    embassy_futures::block_on(clear_paired_phones());
    set_ble_max_bonded_phones(8);

    assert!(embassy_futures::block_on(add_paired_phone(&id(&[9, 9, 9]))));
    assert!(embassy_futures::block_on(add_paired_phone(&id(&[9, 9, 9]))));
    assert_eq!(embassy_futures::block_on(paired_phone_count()), 1);
}

#[test]
fn remove_existing_and_missing_phone() {
    embassy_futures::block_on(clear_paired_phones());
    set_ble_max_bonded_phones(8);
    let device = id(&[7, 7, 7]);

    assert!(embassy_futures::block_on(add_paired_phone(&device)));
    assert!(embassy_futures::block_on(is_phone_paired(&device)));
    assert!(embassy_futures::block_on(remove_paired_phone(&device)));
    assert!(!embassy_futures::block_on(is_phone_paired(&device)));
    assert!(!embassy_futures::block_on(remove_paired_phone(&device)));
}

#[test]
fn clear_returns_removed_count() {
    embassy_futures::block_on(clear_paired_phones());
    set_ble_max_bonded_phones(8);
    assert!(embassy_futures::block_on(add_paired_phone(&id(&[1]))));
    assert!(embassy_futures::block_on(add_paired_phone(&id(&[2]))));
    assert!(embassy_futures::block_on(add_paired_phone(&id(&[3]))));

    let removed = embassy_futures::block_on(clear_paired_phones());
    assert_eq!(removed, 3);
    assert_eq!(embassy_futures::block_on(paired_phone_count()), 0);
}
