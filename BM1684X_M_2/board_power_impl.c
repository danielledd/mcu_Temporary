#include <mp5475.h>
#include <chip.h>
#include <common.h>
#include <mp5475.h>
#include <timer.h>
int pmic_channel_a_on(void)
{
	return mp5475_buck_on(0x90);
}

void pmic_channel_a_off(void)
{
	mp5475_buck_off(0);
}

int pmic_channel_d_on(void)
{
	// mp5475_buck_on(0x0);
	// timer_udelay(58*1000);
	return mp5475_buck_on(0x10);
}

void pmic_channel_d_off(void)
{
	mp5475_buck_off(3);                                                                       
}

int pmic_channel_b_on(void)
{
	return mp5475_buck_on(0xf0);
}

void pmic_channel_b_off(void)
{
	mp5475_buck_off(1);
}

int pmic_channel_c_on(void)
{
	return mp5475_buck_on(0xb0);
}

void pmic_channel_c_off(void)
{
	mp5475_buck_off(2);
}

int sys_rst_deassert_on(void)
{
	chip_enable();
	return 0;
}

void sys_rst_deassert_off(void)
{
}

int sys_rst_assert_on(void)
{
	chip_disable();
	// gpio_clear(PCIEE_RST_X_MCU_PORT,PCIEE_RST_X_MCU_PIN);
	return 0;
}

void sys_rst_assert_off(void)
{
	/* reset chip firstly when power off */
	chip_disable();
}

int check_pcie_reset_on(void)
{
	board_init();
	return 0;
}

void check_pcie_reset_off(void)
{
}

int powerchip_init_on(void)
{
	mp5475_init();

	return 0;
}
