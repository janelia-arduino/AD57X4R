.PHONY: venv
venv:
	rm -rf .venv;\
	mkdir .venv;\
	python3 -m venv .venv;\
	source .venv/bin/activate;\
	pip install platformio

.PHONY: clean
clean:
	rm -rf .pio

firmware: clean
	. .venv/bin/activate; pio run -e teensy41

.PHONY: upload
upload: clean
	. .venv/bin/activate; pio run -e teensy41 --target upload --upload-port /dev/ttyACM0

.PHONY: monitor
monitor:
	. .venv/bin/activate; pio device monitor
