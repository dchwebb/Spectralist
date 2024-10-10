void SAI1_IRQHandler(void)
{
	audioCodec.Interrupt();
}

void USB_DRD_FS_IRQHandler()
{
	usb.InterruptHandler();
}

void SysTick_Handler(void)
{
	++SysTickVal;
}

void NMI_Handler(void)
{
	while (1) {}
}

void HardFault_Handler(void)
{
	while (1) {}
}

void MemManage_Handler(void)
{
	while (1) {}
}

void BusFault_Handler(void)
{
	while (1) {}
}

void UsageFault_Handler(void)
{
	while (1) {}
}




