#include "GeneralAssembler.hpp"

namespace {
	const bool ga_assembled = Factory<Assembler>::GetInstance()->Register("general", [](const json& config) {
		return new GeneralAssembler(config);
	});
}
