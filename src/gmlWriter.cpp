#include "src/headers/gmlWriter.h"

// rather ugly...

bool GMLWriter::write(TransMem * const tm){

    if(tm == nullptr){
        // TODO: error msg
        return false;
    }

    QFile file(QString::fromStdString("TransMemDump.graphml"));

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

    xml.writeStartElement("graph");
    xml.writeAttribute("id", "G");
    xml.writeAttribute("edgedefault", "directed");

    // add frames
    for(Frame f : tm->frames)
       addNode(QString::fromStdString(f.frameID));

    // add links
    for(Link l : tm->links){
        QString parent = QString::fromStdString(l.parent->frameID);
        QString child =  QString::fromStdString(l.child->frameID);
        addEdge(parent+"-"+child, parent, child);
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

void GMLWriter::addNode(const QString &name){

    xml.writeStartElement("node");
    xml.writeAttribute("id", name);
    xml.writeEndElement();  // end node

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
