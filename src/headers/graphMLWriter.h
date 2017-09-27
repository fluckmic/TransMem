#ifndef GRAPHMLWRITER_H
#define GRAPHMLWRITER_H

#include <QXmlStreamWriter>
#include <QFile>
#include "../../src/headers/typedefs.h"

#include "transmem/transmem.h"

class TransMem;

/*! \class GraphMLWriter
 * \brief Can be used to dump the graph representing the current state of the
 * data structure as an GraphML file. (https://en.wikipedia.org/wiki/GraphML)
 *
 * The graph represented by the GraphML file can be displayed for example with Gephi.
 * (https://gephi.org/)
*/
class GraphMLWriter {

public:

    bool write(const QString &path, const TransMem &tm);

protected:

    void addNode(const QString &name);
    void addEdge(const QString &label, const QString &src, const QString &dst);
    void addKey(const QString &id, const QString &attributeTarget, const QString &name, const QString &attributeType);

    QXmlStreamWriter xml;
};

#endif // GRAPHMLWRITER_H
