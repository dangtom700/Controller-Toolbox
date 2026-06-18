#include "controllers.h"
#include "ControllerToolbox.h"   // umbrella include from lib/

namespace residentialbuildingcomfortsmpc {

std::vector<NamedController> makeControllers(double Ts) {
    std::vector<NamedController> out;

    // TODO: replace stubs with the real roster (paper's method must appear).
    // out.push_back({"Ctrl1", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl2", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl3", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl4", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl5", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl6", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl7", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl8", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl9", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl10", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl11", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    // out.push_back({"Ctrl12", std::make_shared<ctrl::DiscretePID>(/*...*/)});
    out.push_back({"OpenLoop", nullptr});  // TODO: real controllers above
    return out;
}

}  // namespace residentialbuildingcomfortsmpc
