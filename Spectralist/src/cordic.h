#pragma once
#include "initialisation.h"
#include <cmath>

class Cordic {
public:

	inline static float ToFloat()
	{
		constexpr float mult = 1.0f / 2147483648.0f;
		return (float)((int)CORDIC->RDATA) * mult;
	}


	inline static constexpr int TrigToQ31(float x)
	{
		while (x > pi) 		x -= pi_x_2;
		while (x < -pi) 	x += pi_x_2;

		// convert float to q1_31 format, dividing by pi
		return (int)(x * (float)M_1_PI * 2147483648.0f);
	}


	static float Sin(float x)
	{
		CORDIC->CSR = (1 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
				(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

		CORDIC->WDATA = TrigToQ31(x);
		return Cordic::ToFloat();
	}


	static float Sin(uint32_t x)						// Use x directly, without conversion to float
	{
		CORDIC->CSR = (1 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
				(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

		CORDIC->WDATA = x;
		return Cordic::ToFloat();
	}


	static float Cos(float x)
	{
		CORDIC->CSR = (0 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
				(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

		CORDIC->WDATA = TrigToQ31(x);
		return Cordic::ToFloat();
	}



	static float Tan(float x)
	{
		CORDIC->CSR = (1 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
				CORDIC_CSR_NRES |						// 2 Results as we need both sin and cos
				(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

		CORDIC->WDATA = TrigToQ31(x);

		// convert values back to floats scaling by * 2 at the same time
		const float sin = (float)((int)CORDIC->RDATA);	// command will block until RDATA is ready - no need to poll RRDY flag
		const float cos = (float)((int)CORDIC->RDATA);
		return sin / cos;
	}


	static float CordicExp(float x)
	{
		// use CORDIC sinh function and generate e^x = sinh(x) + cosh(x)
		CORDIC->CSR = (6 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
				CORDIC_CSR_SCALE_0 |					// Must be 1 for sinh
				CORDIC_CSR_NRES |						// 2 Results as we need both sinh and cosh
				(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

		// convert float to q1_31 format scaling x by 1/2 at the same time
		int q31;
		if (x < -1.118f) {
			q31 = (int)((x + 1.0f) * 1073741824.0f);	// as range of x is limited to -1.118 to +1.118 reduce exponent by e^-1 (note that only values from around -1.75 to 0 used in this mechanism)
		} else {
			q31 = (int)(x * 1073741824.0f);
		}

		//volatile float etest = std::exp(x);

		CORDIC->WDATA = q31;

		// convert values back to floats scaling by * 2 at the same time
		const float sinh = (float)((int)CORDIC->RDATA) / 1073741824.0f;	// command will block until RDATA is ready - no need to poll RRDY flag
		const float cosh = (float)((int)CORDIC->RDATA) / 1073741824.0f;
		const float res = sinh + cosh;
		if (x < -1.118f) {
			return res * 0.3678794411714f;				// multiply by e^-1 to correct range offset
		} else {
			return res;
		}
	}
};


