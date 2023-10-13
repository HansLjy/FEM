#pragma once

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <iostream>
#include <map>
#include "JsonUtil.h"

struct Object final {
	std::string _type;
	std::shared_ptr<void> _ptr;

	friend class Creator;
private:
	Object(const std::string& type, void* ptr);
};

class Deleter {
public:
	using ProductDeleter = std::function<void(void*)>;

	static Deleter* GetInstance() {
		if (!_the_deletor) {
			_the_deletor = new Deleter;
		}
		return _the_deletor;
	}

	bool Register(const std::string& name, const ProductDeleter& deleter) {
		std::cerr << name << " deleter registered" << std::endl;
		return _deleter_map.insert(std::make_pair(name, deleter)).second;
	}

	void Delete(const std::string& name, void* ptr) {
		return _deleter_map[name](ptr);
	}

private:
	static Deleter* _the_deletor;
	std::map<std::string, ProductDeleter> _deleter_map;
};

namespace TypeErasure {
	template<class T>
	bool RegisterForDeleter(const std::string& type) {
		return Deleter::GetInstance()->Register(type, [](void* ptr) {
			delete static_cast<T*>(ptr);
		});
	}
}

class Creator {
public:
	using ProductCreator = std::function<void*(const json& config)>;

	static Creator* GetInstance() {
		if (!_the_creator) {
			_the_creator = new Creator;
		}
		return _the_creator;
	}

	bool Register(const std::string& name, const ProductCreator& creator) {
		std::cerr << name << " creator registered" << std::endl;
		return _creator_map.insert(std::make_pair(name, creator)).second;
	}

	Object GetProduct(const std::string& type, const json& config) {
		return {
			type,
			_creator_map[type](config)
		};
	}

private:
	static Creator* _the_creator;
	std::map<std::string, ProductCreator> _creator_map;
};

namespace CreatorRegistration {
	template<class T>
	bool RegisterForCreator(const std::string& type) {
		return Creator::GetInstance()->Register(type, [](const json& config) {
			return new T(config);
		});
	}
}


template <class Product>
class Caster {
public:
	using ProductCaster = std::function<Product(void*)>;

	static Caster* GetInstance() {
		if (!_the_factory) {
			_the_factory = new Caster;
		}
		return _the_factory;
	}

	bool Register(const std::string& name, const ProductCaster& creator) {
		std::cerr << name << " registered" << std::endl;
		return _creator_map.insert(std::make_pair(name, creator)).second;
	}

	Product Cast(const Object& obj) {
		return _creator_map[obj._type](obj._ptr.get());
	}

private:
	static Caster* _the_factory;
	std::map<std::string, ProductCaster> _creator_map;
};

namespace CasterRegistration {
	template<class Interface, class T>
	bool RegisterForCaster(const std::string& type) {
		return Caster<Interface>::GetInstance()->Register(type, [](void* ptr) {
			return Interface(static_cast<T*>(ptr));
		});
	}
}


namespace TypeErasure {
	template<class Interface>
	void Cast2Interface(
		const typename std::vector<Object>::const_iterator& begin,
		const typename std::vector<Object>::const_iterator& end,
		std::vector<Interface>& out
	) {
		for (auto itr = begin; itr != end; itr++) {
			out.emplace_back(Caster<Interface>::GetInstance()->Cast(*itr));
		}
	}
}

namespace TypeErasure {
	inline void ReadObjects(const json& config, std::vector<Object>& objs) {
		for (const auto& object_config : config) {
			objs.push_back(Creator::GetInstance()->GetProduct(object_config["type"], object_config));
		}
	}
}

template<class Interface>
class InterfaceContainer {
public:
	virtual void BindObjects (
		const typename std::vector<Object>::const_iterator& begin,
		const typename std::vector<Object>::const_iterator& end
	) {
		_objs.clear();
		TypeErasure::Cast2Interface(begin, end, _objs);
	}

	void BindObjects(const std::vector<Object>& objs) {
		BindObjects(objs.begin(), objs.end());
	}

	std::vector<Interface> _objs;
};