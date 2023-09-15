#include "System.hpp"
#include "Pattern.h"

template<>
Factory<System>* Factory<System>::_the_factory = nullptr;