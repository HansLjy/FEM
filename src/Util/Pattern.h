//
// Created by hansljy on 2022/2/22.
//

#ifndef FEM_PATTERN_H
#define FEM_PATTERN_H

#include <iostream>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
using nlohmann::json;

template <class Product>
class Factory {
public:
	using ProductCreator = std::function<Product*(const json& config)>;
	using StatelessProductCreator = std::function<Product*()>;

	static Factory* GetInstance() {
		if (!_the_factory) {
			_the_factory = new Factory;
		}
		return _the_factory;
	}

	bool Register(const std::string& name, const ProductCreator& creator) {
		bool result = _creator_map.insert(std::make_pair(name, creator)).second;
		if (result) {
			std::cerr << name << " registered" << std::endl;
		} else {
			std::cerr << name << " can't be registered" << std::endl;
		}
		return result;
	}

	bool Register(const std::string& name, const StatelessProductCreator& creator) {
		bool result = _stateless_creator_map.insert(std::make_pair(name, creator)).second;
		if (result) {
			std::cerr << name << " registered" << std::endl;
		} else {
			std::cerr << name << " can't be registered" << std::endl;
		}
		return result;
	}

	Product* GetProduct(const std::string& name, const json& config) {
		if (_creator_map.find(name) == _creator_map.end()) {
			spdlog::error("Unknown product name {} detected", name);
		}
		return _creator_map[name](config);
	}

	Product* GetProduct(const std::string& name) {
		if (_stateless_creator_map.find(name) == _stateless_creator_map.end()) {
			spdlog::error("Unknown product name {} detected", name);
		}
		return _stateless_creator_map[name]();
	}

private:
	static Factory* _the_factory;
	std::map<std::string, ProductCreator> _creator_map;
	std::map<std::string, StatelessProductCreator> _stateless_creator_map;
};

#define DEFINE_HAS_MEMBER(MEMBER_NAME)                                         \
    template <typename T>                                                      \
    class has_member_##MEMBER_NAME {                                           \
        typedef char yes_type;                                                 \
        typedef long no_type;                                                  \
        template <typename U> static yes_type test(decltype(&U::member_name)); \
        template <typename U> static no_type  test(...);                       \
    public:                                                                    \
        static constexpr bool value = sizeof(test<T>(0)) == sizeof(yes_type);  \
    };

#define HAS_MEMBER(CLASS_NAME, MEMBER_NAME) (has_member_##MEMBER_NAME<CLASS_NAME>::value)


/* Concept-Model idiom */
#define CONCEPT_MODEL_IDIOM_BEGIN(InterfaceName) \
class InterfaceName {\
public:\
	template<class T>\
	InterfaceName(T* model) : _concept(new Model<T>(model)) {}

#define CONCEPT_MODEL_IDIOM_CONCEPT\
	class Concept {\
	public:

#define CONCEPT_MODEL_IDIOM_MODEL\
		virtual ~Concept() = default;\
	};\
	\
	template<class T>\
	class Model : public Concept {\
	public:\
		Model(T* obj) : _obj(obj) {}


#define CONCEPT_MODEL_IDIOM_END\
	private:\
		T* _obj;\
	};\
	\
	std::shared_ptr<Concept> _concept;\
};

#define ADD_INTERFACE_FUNCTION(FUNCTION_SIGNATURE, FUNCTION_CALL)\
	FUNCTION_SIGNATURE {\
		return _concept->FUNCTION_CALL;\
	}

#define ADD_MODEL_FUNCTION(FUNCTION_SIGNATURE, FUNCTION_CALL)\
	FUNCTION_SIGNATURE override { \
		return _obj->FUNCTION_CALL;\
	}

#define ADD_CONCEPT_FUNCTION(FUNCTION_SIGNATURE)\
	virtual FUNCTION_SIGNATURE = 0;

#endif //FEM_PATTERN_H