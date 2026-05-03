from host.rp2040_swim import (
    ROP_DESTRUCTIVE_WARNING,
    STM8_ROP_ENABLED_VALUE,
    main,
    rop_is_enabled,
)


def test_rop_decode_enabled_only_for_0xaa() -> None:
    assert rop_is_enabled(STM8_ROP_ENABLED_VALUE)
    assert not rop_is_enabled(0x00)
    assert not rop_is_enabled(0x55)
    assert not rop_is_enabled(0xFF)


def test_set_rop_requires_confirmation(capsys) -> None:
    rc = main(["set-rop", "--device", "stm8s003f3"])
    captured = capsys.readouterr()

    assert rc == 1
    assert "--yes-i-know" in captured.err
    assert "refusing" in captured.err


def test_unprotect_rop_requires_destructive_confirmation(capsys) -> None:
    rc = main(["unprotect-rop", "--device", "stm8s003f3"])
    captured = capsys.readouterr()

    assert rc == 1
    assert "--yes-erase-all" in captured.err
    assert "mass erase" in captured.err
    assert "program flash" in captured.err
    assert "data EEPROM" in captured.err
    assert "option bytes" in captured.err


def test_destructive_warning_names_all_erased_areas() -> None:
    assert "program flash" in ROP_DESTRUCTIVE_WARNING
    assert "data EEPROM" in ROP_DESTRUCTIVE_WARNING
    assert "option bytes" in ROP_DESTRUCTIVE_WARNING
