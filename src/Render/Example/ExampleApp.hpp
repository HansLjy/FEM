#ifndef EXAMPLE_APP_HPP
#define EXAMPLE_APP_HPP

#include "GUI/GUI.hpp"

class ExampleApp : public GUI {
public:
    void InitializeScene(Scene &scene) override;
    void Processing(Scene &scene) override;

private:
    int id1, id2;
};

#endif