All Driver related work

goes in Linux to folder drivers/staging/enclustra/

add the following line to drivers/staging/Kconfig

    source "drivers/staging/enclustra/Kconfig"

add the following line to drivers/staging/Makefile

    obj-$(CONFIG_ENCLUSTRA) += enclustra/

add the follwing parts to the devicetree

//	amba section
	tft@43C10000 {
    	compatible = "enclustra_fb";
    	reg = <0x43C10000 0x10000>;	
	};


