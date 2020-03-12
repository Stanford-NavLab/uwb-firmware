#include "Complex.h"

static double abss(struct Complex *this) {
	return this->re*this->re+this->im*this->im;
}
static struct Complex new(double real, double imag) {
	return (struct Complex){.re=real, .im=imag, .abss=&abss};
}
const struct ComplexClass Complex={.new=&new};
