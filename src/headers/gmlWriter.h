#ifndef GMLWRITER_H
#define GMLWRITER_H

#include <QXmlStreamWriter>
#include <QFile>
#include <typedefs.h>

#include "transmem/transmem.h"

class TransMem;

class GMLWriter {

public:

    bool write(TransMem * const tm);

protected:

    void addNode(const QString &name);
    void addEdge(const QString &label, const QString &src, const QString &dst);
    void addKey(const QString &id, const QString &attributeTarget, const QString &name, const QString &attributeType);

    QXmlStreamWriter xml;
};


#endif // GMLWRITER_H
