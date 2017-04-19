#include "stampedtransformation.h"

using namespace std;
using namespace std::chrono;

ostream &operator<<(ostream &os, const StampedTransformation &te){

    milliseconds ms = duration_cast<milliseconds>(te.time.time_since_epoch());
    time_t yet = system_clock::to_time_t(te.time);

    cout << "\n++++++++++++++++++++++++++ \n";
    cout << "  timestamp:  " << put_time(localtime(&yet), "%T") << ":" << ms.count() % 1000 << "\n";
    cout << "   rotation: (" << te.rotation.scalar() << "," << te.rotation.x() << "," << te.rotation.y() << "," << te.rotation.z() << ")\n";
    cout << "translation: (" << te.translation.scalar() << "," << te.translation.x() << "," << te.translation.y() << "," << te.translation.z() << ")\n";
    cout << "++++++++++++++++++++++++++ \n";

    return os;
}


