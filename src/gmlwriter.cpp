#include "gmlwriter.h"

bool GMLWriter::write(TransMem * const tm){

    if(tm == nullptr){
        // TODO: error msg
        return false;
    }

    QFile file(QString::fromStdString(path));

    if(!file.open(QFile::WriteOnly | QFile::Text)){
        // TODO: error msg
        return false;
    }

    xml.setDevice(&file);
    xml.setAutoFormatting(true);

    xml.writeStartDocument();
    xml.writeStartElement("graphml");
    xml.writeNamespace("http://graphml.graphdrawing.org/xmlns");

    addKey("k0", "node", "nodeType", "string");
    addKey("k1", "node", "timestamp", "string");
    addKey("k2", "node", "rotation", "string");
    addKey("k3", "node", "translation", "string");

    xml.writeStartElement("graph");
    xml.writeAttribute("id", "G");
    xml.writeAttribute("edgedefault", "directed");

    // add frames
    for(Frame f : tm->frames)
       addFrameNode(QString::fromStdString(f.frameID));

    // add links
    for(Link l : tm->links){
        QString parent = QString::fromStdString(l.parent->frameID);
        QString child = QString::fromStdString(l.child->frameID);
        QString edge = "b-"+parent+"-"+child;

        addBufferNode(edge);
        addEdge(edge+"-p", parent, edge);
        addEdge(edge+"-c", edge, child);

        // add buffer entries
        unsigned int indx = 0;
        QString oldN = edge;
        QString newN = edge+"-"+QString::number(indx);
        for(StampedTransformation s: l.buf.buffer){
            addEntryNode(newN, s);
            addEdge(oldN+"-"+newN, oldN, newN);
            oldN = newN;
            newN = edge+"-"+QString::number(++indx);
        }

    }

    xml.writeEndElement();      // end graph
    xml.writeEndElement();      // end graphml
    xml.writeEndDocument();

    file.close();
    if(file.error()){
        // TODO: error msg
        return false;
    }

    return true;
}

void GMLWriter::addFrameNode(const QString &name){

    xml.writeStartElement("node");
    xml.writeAttribute("id", name);
    xml.writeStartElement("data");
    xml.writeAttribute("key", "k0");
    xml.writeCharacters("Frame");
    xml.writeEndElement();  // end data
    xml.writeEndElement();  // end node

}
void GMLWriter::addBufferNode(const QString &name){

    xml.writeStartElement("node");
    xml.writeAttribute("id", name);
    xml.writeStartElement("data");
    xml.writeAttribute("key", "k0");
    xml.writeCharacters("Buffer");
    xml.writeEndElement();  // end data
    xml.writeEndElement();  // end node
}
void GMLWriter::addEntryNode(const QString &name, const StampedTransformation &s){

    xml.writeStartElement("node");
    xml.writeAttribute("id", name);
    xml.writeStartElement("data");
    xml.writeAttribute("key", "k0");
    xml.writeCharacters("Entry");
    xml.writeEndElement();  // end node type data
    xml.writeStartElement("data");
    xml.writeAttribute("key", "k1");
    xml.writeCharacters(QString::fromStdString(s.timeAsString()));
    xml.writeEndElement();  // end time data
    xml.writeStartElement("data");
    xml.writeAttribute("key", "k2");
    xml.writeCharacters(QString::fromStdString(s.rotationAsString()));
    xml.writeEndElement();  // end rotation data
    xml.writeStartElement("data");
    xml.writeAttribute("key", "k3");
    xml.writeCharacters(QString::fromStdString(s.translationAsString()));
    xml.writeEndElement();  // end translation data
    xml.writeEndElement();  // end node

    return;
}

void GMLWriter::addEdge(const QString &label, const QString &src, const QString &dst){

    xml.writeStartElement("edge");
    xml.writeAttribute("id", label);
    xml.writeAttribute("source", src);
    xml.writeAttribute("target", dst);
    xml.writeEndElement();

    return;
}

void GMLWriter::addKey(const QString &id, const QString &attributeTarget, const QString &name, const QString &attributeType){

    xml.writeStartElement("key");
    xml.writeAttribute("id", id);
    xml.writeAttribute("for", attributeTarget);
    xml.writeAttribute("attr.name", name);
    xml.writeAttribute("attr.type", attributeType);
    xml.writeEndElement();

    return;
}
