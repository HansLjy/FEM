#ifndef EXAMPLE_APP_HPP
#define EXAMPLE_APP_HPP

#include "GUI/GUI.hpp"

class ExampleApp : public GUI {
public:
	ExampleApp() : GUI(true) {}
    void InitializeScene(Scene &scene) override;
    void Processing(Scene &scene) override;
};

#endif