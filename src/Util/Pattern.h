//
// Created by hansljy on 2022/2/22.
//

#ifndef FEM_PATTERN_H
#define FEM_PATTERN_H

#include <iostream>
#include <nlohmann/json.hpp>
using nlohmann::json;

template <class Product>
class Factory {
public:
	using ProductCreator = std::function<Product*(const json& config)>;

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

	Product* GetProduct(const std::string& name, const json& config) {
		return _creator_map[name](config);
	}

private:
	static Factory* _the_factory;
	std::map<std::string, ProductCreator> _creator_map;
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

#endif //FEM_PATTERN_H