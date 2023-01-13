//
// Created by hansljy on 10/13/22.
//

#include "Simulator.h"
#include "Player.h"
#include "spdlog/spdlog.h"

int main(const int argc, const char* argv[]) {
   if (argc <= 1) {
       Simulator simulator("FEM Simulator");
       simulator.LoadScene(CONFIG_PATH "/simulator.json");
       simulator.MainLoop();
	} else {
		if (std::string(argv[1]) == "simulate") {
			if (argc != 3) {
				spdlog::info("You should provide the output dir of the simulator");
			} else {
				Simulator simulator("FEM simulator");
				simulator.LoadScene(CONFIG_PATH "/simulator.json");
				simulator.Simulate(std::string(OUTPUT_PATH) + "/" + argv[2]);
				// simulator.Simulate(std::string(OUTPUT_PATH) + "/test");
			}
		} else if (std::string(argv[1]) == "play") {
			Player player;
			player.LoadAnimation(CONFIG_PATH "/player.json");
			player.MainLoop();
		}
	}
}