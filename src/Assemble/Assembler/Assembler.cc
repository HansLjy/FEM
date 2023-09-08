#include "Assembler.hpp"


void Assembler::BindObjects(const typename std::vector<Object *>::const_iterator &begin, const typename std::vector<Object *>::const_iterator &end) {
	for (auto itr = begin; itr != end; ++itr) {
		_objs.push_back((*itr));
	}
}

void Assembler::BindObjects(const std::vector<Object*>& objs) {
	BindObjects(objs.begin(), objs.end());
}

void Assembler::BindSystem(System &system) {
	BindObjects(system.GetObjs());
}
