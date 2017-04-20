#include "stampedtransformation.h"

using namespace std;
using namespace std::chrono;

ostream &operator<<(ostream &os, const StampedTransformation &te){

    cout << "\n++++++++++++++++++++++++++ \n";
    cout << "  timestamp:  " << te.timeAsString() << "\n";
    cout << "   rotation: (" << te.rotationAsString() << "\n";
    cout << "translation: (" << te.translationAsString() << "\n";
    cout << "++++++++++++++++++++++++++ \n";

    return os;
}


std::string StampedTransformation::timeAsString() const {

    milliseconds ms = duration_cast<milliseconds>(time.time_since_epoch());
    time_t yet = system_clock::to_time_t(time);

    ostringstream oss;
    oss << put_time(localtime(&yet), "%T") << ":" << std::to_string(ms.count() % 1000);

    return oss.str();

}

std::string StampedTransformation::rotationAsString() const {

    ostringstream oss;
    oss << "(" << rotation.scalar() << "," << rotation.x() << "," << rotation.y() << "," << rotation.z() << ")";
    return oss.str();

}

std::string StampedTransformation::translationAsString() const {

    ostringstream oss;
    oss << "(" << translation.scalar() << "," << translation.x() << "," << translation.y() << "," << translation.z() << ")";

    return oss.str();
}
