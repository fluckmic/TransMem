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


void StampedTransformation::writeJSON(QJsonObject &json) const {

    QStringList timeList = (QString::fromStdString(timeAsString())).split(":");

    // NOTE: assertion just during development
    // list should always contain 4 elements
    assert(timeList.length() > 3);

    QJsonObject timeObject; timeObject.insert("1_H", timeList.at(0));
                            timeObject.insert("2_m", timeList.at(1));
                            timeObject.insert("3_s", timeList.at(2));
                            timeObject.insert("4_ms", timeList.at(3));

    QJsonObject rotObject; rotObject.insert("1_s", rotation.scalar());
                           rotObject.insert("2_x", translation.x());
                           rotObject.insert("3_y", translation.y());
                           rotObject.insert("4_z", translation.z());

    QJsonObject transObject; transObject.insert("1_s", translation.scalar());
                             transObject.insert("2_x", translation.x());
                             transObject.insert("3_y", translation.y());
                             transObject.insert("4_z", translation.z());

    json.insert("1_timestamp", timeObject);
    json.insert("2_rotation", rotObject);
    json.insert("3_translation", transObject);
}
