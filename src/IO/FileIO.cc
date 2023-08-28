#include "FileIO.hpp"
#include "Pattern.h"
#include "ObjIO.hpp"

template<>
Factory<FileIO>* Factory<FileIO>::_the_factory = nullptr;

namespace {
	const bool obj_io_registered = Factory<FileIO>::GetInstance()->Register("obj", []() {
		return new ObjIO;
	});
}