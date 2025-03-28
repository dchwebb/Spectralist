#pragma once

class GpioPin {
public:
	enum class Type {Input, InputPullup, Output, AlternateFunction};
	enum class DriveStrength {Low, Medium, High, VeryHigh};

	GpioPin(GPIO_TypeDef* port, uint32_t pin, Type pinType, uint32_t alternateFunction = 0, DriveStrength driveStrength = DriveStrength::Low) :
		port(port), pin(pin), pinType(pinType)
	{
		Init(port, pin, pinType, alternateFunction, driveStrength);		// Init function is static so can be called without instantiating object
	}


	static void Init(GPIO_TypeDef* port, uint32_t pin, Type pinType, uint32_t alternateFunction = 0, DriveStrength driveStrength = DriveStrength::Low)
	{
		// maths to calculate RCC clock to enable
		uint32_t portPos = ((uint32_t)port - AHB2PERIPH_BASE_NS) >> 10;
		RCC->AHB2ENR |= (1 << portPos);

		// 00: Input, 01: Output, 10: Alternate function, 11: Analog (reset state)
		if (pinType == Type::Input || pinType == Type::InputPullup) {
			port->MODER &= ~(0b11 << (pin * 2));
			if (pinType == Type::InputPullup) {
				port->PUPDR |= (0b1 << (pin * 2));
			}

		} else if (pinType == Type::Output) {
			port->MODER |=  (0b01 << (pin * 2));
			port->MODER &= ~(0b10 << (pin * 2));

		} if (pinType == Type::AlternateFunction) {
			port->MODER |=  (0b10 << (pin * 2));
			port->MODER &= ~(0b01 << (pin * 2));
			if (pin < 8) {
				port->AFR[0] |= alternateFunction << (pin * 4);
			} else {
				port->AFR[1] |= alternateFunction << ((pin - 8) * 4);
			}
		}

		port->OSPEEDR |= static_cast<uint32_t>(driveStrength) << (pin * 2);
	}


	bool IsHigh() {
		return (port->IDR & (1 << pin));
	}

	bool IsLow() {
		return ((port->IDR & (1 << pin)) == 0);
	}

	static void SetHigh(GPIO_TypeDef* port, uint32_t pin) {
		port->ODR |= (1 << pin);
	}

	void SetHigh() {
		port->ODR |= (1 << pin);
	}

	static void SetLow(GPIO_TypeDef* port, uint32_t pin) {
		port->ODR &= ~(1 << pin);
	}

	void SetLow() {
		port->ODR &= ~(1 << pin);
	}

private:
	GPIO_TypeDef* port;
	uint32_t pin;
	Type pinType;
};
