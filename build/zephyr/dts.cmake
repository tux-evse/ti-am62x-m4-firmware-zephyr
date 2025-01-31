add_custom_target(devicetree_target)

set_target_properties(devicetree_target PROPERTIES "DT_CHOSEN|zephyr,sram" "/memory@0")
set_target_properties(devicetree_target PROPERTIES "DT_CHOSEN|zephyr,console" "/serial@4a00000")
set_target_properties(devicetree_target PROPERTIES "DT_CHOSEN|zephyr,shell-uart" "/serial@4a00000")
set_target_properties(devicetree_target PROPERTIES "DT_CHOSEN|zephyr,sram1" "/memory@9CC00000")
set_target_properties(devicetree_target PROPERTIES "DT_CHOSEN|zephyr,ipc_shm" "/reserved-memory/memory@9cb00000")
set_target_properties(devicetree_target PROPERTIES "DT_CHOSEN|zephyr,ipc" "/mailbox0@A9000000")
set_target_properties(devicetree_target PROPERTIES "DT_ALIAS|led0" "/leds/led_0")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/|compatible" "phytec,phyboard-lyra-am62xx-m4;ti,am625;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/chosen" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_REG|/chosen|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/chosen|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/chosen|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/aliases" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_REG|/aliases|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/aliases|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/aliases|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/soc" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc|compatible" "simple-bus;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc|ranges" "None")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/soc/interrupt-controller@e000e100" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|nvic" "/soc/interrupt-controller@e000e100")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/interrupt-controller@e000e100|reg" "3758153984;3072;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/interrupt-controller@e000e100|arm,num-irq-priority-bits" "3")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/interrupt-controller@e000e100|interrupt-controller" "True")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/interrupt-controller@e000e100|compatible" "arm,v7m-nvic;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/interrupt-controller@e000e100|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/interrupt-controller@e000e100|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/interrupt-controller@e000e100|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc/interrupt-controller@e000e100|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc/interrupt-controller@e000e100|ADDR" "0xe000e100;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc/interrupt-controller@e000e100|SIZE" "0xc00;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/soc/timer@e000e010" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|systick" "/soc/timer@e000e010")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/timer@e000e010|reg" "3758153744;16;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/timer@e000e010|status" "okay")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/timer@e000e010|compatible" "arm,armv7m-systick;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/timer@e000e010|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/timer@e000e010|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/soc/timer@e000e010|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc/timer@e000e010|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc/timer@e000e010|ADDR" "0xe000e010;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/soc/timer@e000e010|SIZE" "0x10;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/cpus" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_REG|/cpus|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/cpus|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/cpus|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/cpus/cpu@0" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|cpu0" "/cpus/cpu@0")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/cpus/cpu@0|clock-frequency" "400000000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/cpus/cpu@0|status" "okay")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/cpus/cpu@0|compatible" "arm,cortex-m4f;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/cpus/cpu@0|reg" "0;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/cpus/cpu@0|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/cpus/cpu@0|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/cpus/cpu@0|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/cpus/cpu@0|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/cpus/cpu@0|ADDR" "0x0;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/cpus/cpu@0|SIZE" "NONE;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/memory@0" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|sram0" "/memory@0")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@0|reg" "0;196608;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@0|compatible" "mmio-sram;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@0|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@0|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@0|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory@0|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory@0|ADDR" "0x0;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory@0|SIZE" "0x30000;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/memory1@40000" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|sram1" "/memory1@40000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory1@40000|reg" "262144;65536;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory1@40000|compatible" "mmio-sram;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory1@40000|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory1@40000|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory1@40000|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory1@40000|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory1@40000|ADDR" "0x40000;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory1@40000|SIZE" "0x10000;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/system-clock" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|sysclk" "/system-clock")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/system-clock|clock-frequency" "400000000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/system-clock|compatible" "fixed-clock;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/system-clock|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/system-clock|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/system-clock|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/system-clock|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/system-clock|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/system-clock|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/mailbox0@A9000000" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|mbox0" "/mailbox0@A9000000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|reg" "2835349504;512;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|interrupts" "50;4;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|usr-id" "2")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|status" "okay")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|compatible" "ti,omap-mailbox;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mailbox0@A9000000|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/mailbox0@A9000000|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/mailbox0@A9000000|ADDR" "0xa9000000;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/mailbox0@A9000000|SIZE" "0x200;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/pinctrl@4084000" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|pinctrl" "/pinctrl@4084000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000|reg" "67649536;136;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000|status" "okay")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000|compatible" "ti,k3-pinctrl;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000|ADDR" "0x4084000;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000|SIZE" "0x88;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/pinctrl@4084000/mcu_uart0_rx_default" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|mcu_uart0_rx_default" "/pinctrl@4084000/mcu_uart0_rx_default")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000/mcu_uart0_rx_default|pinmux" "20;327680;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_uart0_rx_default|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_uart0_rx_default|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_uart0_rx_default|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/pinctrl@4084000/mcu_uart0_tx_default" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|mcu_uart0_tx_default" "/pinctrl@4084000/mcu_uart0_tx_default")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000/mcu_uart0_tx_default|pinmux" "24;65536;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_uart0_tx_default|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_uart0_tx_default|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_uart0_tx_default|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/pinctrl@4084000/mcu_gpio0_led_default" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|mcu_gpio0_led_default" "/pinctrl@4084000/mcu_gpio0_led_default")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/pinctrl@4084000/mcu_gpio0_led_default|pinmux" "0;65543;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_gpio0_led_default|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_gpio0_led_default|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/pinctrl@4084000/mcu_gpio0_led_default|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/serial@4a00000" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|uart0" "/serial@4a00000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|reg-shift" "2")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|io-mapped" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|clock-frequency" "48000000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|current-speed" "115200")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|hw-flow-control" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|status" "okay")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|compatible" "ns16550;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|reg" "77594624;512;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|interrupts" "24;4;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/serial@4a00000|pinctrl-names" "default;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/serial@4a00000|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/serial@4a00000|ADDR" "0x4a00000;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/serial@4a00000|SIZE" "0x200;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/gpio@4201010" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|gpio0" "/gpio@4201010")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|reg" "69210128;256;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|status" "okay")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|compatible" "ti,davinci-gpio;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|gpio-controller" "True")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|ngpios" "24")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/gpio@4201010|pinctrl-names" "default;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/gpio@4201010|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/gpio@4201010|ADDR" "0x4201010;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/gpio@4201010|SIZE" "0x100;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/memory@9CC00000" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|ddr0" "/memory@9CC00000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@9CC00000|zephyr,memory-region" "DDR")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@9CC00000|compatible" "zephyr,memory-region;mmio-sram;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@9CC00000|reg" "2629828608;4096;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@9CC00000|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@9CC00000|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/memory@9CC00000|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory@9CC00000|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory@9CC00000|ADDR" "0x9cc00000;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/memory@9CC00000|SIZE" "0x1000;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/reserved-memory" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/reserved-memory|ranges" "None")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/reserved-memory|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/reserved-memory|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/reserved-memory|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/reserved-memory/memory@9cb00000" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|mcu_m4fss_dma_memory_region" "/reserved-memory/memory@9cb00000")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/reserved-memory/memory@9cb00000|compatible" "shared-dma-pool;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/reserved-memory/memory@9cb00000|reg" "2628780032;1048576;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/reserved-memory/memory@9cb00000|NUM" "1")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/reserved-memory/memory@9cb00000|ADDR" "0x9cb00000;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/reserved-memory/memory@9cb00000|SIZE" "0x100000;")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/mbox-consumer" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mbox-consumer|mbox-names" "tx;rx;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mbox-consumer|compatible" "vnd,mbox-consumer;")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mbox-consumer|zephyr,deferred-init" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mbox-consumer|wakeup-source" "False")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/mbox-consumer|zephyr,pm-device-runtime-auto" "False")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/mbox-consumer|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/mbox-consumer|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/mbox-consumer|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/leds" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|leds" "/leds")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/leds|compatible" "gpio-leds;")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/leds|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/leds|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/leds|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_NODE|/leds/led_0" TRUE)
set_target_properties(devicetree_target PROPERTIES "DT_NODELABEL|heartbeat_led" "/leds/led_0")
set_target_properties(devicetree_target PROPERTIES "DT_PROP|/leds/led_0|label" "Heartbeat LED")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/leds/led_0|NUM" "0")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/leds/led_0|ADDR" "")
set_target_properties(devicetree_target PROPERTIES "DT_REG|/leds/led_0|SIZE" "")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|phytec,phyboard-lyra-am62xx-m4" "/")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|ti,am625" "/")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|simple-bus" "/soc")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|arm,v7m-nvic" "/soc/interrupt-controller@e000e100")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|arm,armv7m-systick" "/soc/timer@e000e010")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|arm,cortex-m4f" "/cpus/cpu@0")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|mmio-sram" "/memory@0;/memory1@40000;/memory@9CC00000")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|fixed-clock" "/system-clock")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|ti,omap-mailbox" "/mailbox0@A9000000")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|ti,k3-pinctrl" "/pinctrl@4084000")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|ns16550" "/serial@4a00000")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|ti,davinci-gpio" "/gpio@4201010")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|zephyr,memory-region" "/memory@9CC00000")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|shared-dma-pool" "/reserved-memory/memory@9cb00000")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|vnd,mbox-consumer" "/mbox-consumer")
set_target_properties(devicetree_target PROPERTIES "DT_COMP|gpio-leds" "/leds")
