#include "../src/headers/stampedTransformation.h"

void StampedTransformation::writeJSON(QJsonObject &json) const {

    using namespace std::chrono;

    hours H = duration_cast<hours>(time.time_since_epoch());
    minutes m = duration_cast<minutes>(time.time_since_epoch());
    seconds s = duration_cast<seconds>(time.time_since_epoch());
    milliseconds ms = duration_cast<milliseconds>(time.time_since_epoch());

    QJsonObject timeObject; timeObject.insert("1_H", QString::number(H.count()%24));
                            timeObject.insert("2_m", QString::number(m.count()%60));
                            timeObject.insert("3_s", QString::number(s.count()%60));
                            timeObject.insert("4_ms", QString::number(ms.count()%1000));

    QJsonObject rotObject; rotObject.insert("1_s", QString::number(rotation.scalar()));
                           rotObject.insert("2_x", QString::number(rotation.x()));
                           rotObject.insert("3_y", QString::number(rotation.y()));
                           rotObject.insert("4_z", QString::number(rotation.z()));

    QJsonObject transObject; transObject.insert("1_s", QString::number(translation.scalar()));
                             transObject.insert("2_x", QString::number(translation.x()));
                             transObject.insert("3_y", QString::number(translation.y()));
                             transObject.insert("4_z", QString::number(translation.z()));

    json.insert("1_timestamp", timeObject);
    json.insert("2_rotation", rotObject);
    json.insert("3_translation", transObject);
}
