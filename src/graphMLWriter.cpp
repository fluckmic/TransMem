#include "src/headers/graphMLWriter.h"

bool GraphMLWriter::write(const QString &path, const TransMem &tm) {

    QDateTime currentTime = QDateTime::currentDateTime();
    QString suffixFilename = "_graphML_dump.graphml";

    QFile file(path + currentTime.toString("ddMMyy_HHmmss") + suffixFilename);

    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << file.errorString();
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
    for(auto f : tm.frameID2Frame)
       addNode(QString::fromStdString(f.second.frameID));

    // add links
    for(Link l : tm.links){
        QString parent = QString::fromStdString(l.parent->frameID);
        QString child =  QString::fromStdString(l.child->frameID);
        addEdge(parent+"-"+child, parent, child);
       }

    xml.writeEndElement();      // end graph
    xml.writeEndElement();      // end graphml
    xml.writeEndDocument();

    file.close();
    if(file.error()){
        qDebug() << file.errorString();
        return false;
    }

    return true;
}

void GraphMLWriter::addNode(const QString &name){

    xml.writeStartElement("node");
    xml.writeAttribute("id", name);
    xml.writeEndElement();  // end node

}
void GraphMLWriter::addEdge(const QString &label, const QString &src, const QString &dst){

    xml.writeStartElement("edge");
    xml.writeAttribute("id", label);
    xml.writeAttribute("source", src);
    xml.writeAttribute("target", dst);
    xml.writeEndElement();

    return;
}
void GraphMLWriter::addKey(const QString &id, const QString &attributeTarget, const QString &name, const QString &attributeType){

    xml.writeStartElement("key");
    xml.writeAttribute("id", id);
    xml.writeAttribute("for", attributeTarget);
    xml.writeAttribute("attr.name", name);
    xml.writeAttribute("attr.type", attributeType);
    xml.writeEndElement();

    return;
}
