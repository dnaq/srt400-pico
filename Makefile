all:
	cargo build --release
	until udisksctl mount -b /dev/sdb1; do sleep 1; done && cargo run --release
