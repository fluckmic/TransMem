#ifndef TRANSMEM_H
#define TRANSMEM_H

#include <QtGui/QMatrix4x4>
#include <QtGui/QQuaternion>
#include <unordered_map>
#include <deque>
#include <queue>
#include <mutex>
#include <functional>
#include <QDateTime>
#include <cmath>

#include "../../src/headers/typedefs.h"
#include "../../src/headers/frameAndLink.h"
#include "../../src/headers/stampedTransformation.h"
#include "../../src/headers/graphMLWriter.h"

class NoSuchLinkFoundException : public std::runtime_error {

public:
    NoSuchLinkFoundException(const FrameID &_srcFrame, const FrameID &_destFrame)
    : std::runtime_error("No such link found: " + _srcFrame + "-" + _destFrame)
    , srcFrame(_srcFrame)
    , destFrame(_destFrame)
    {}

    virtual const char* what() const throw();

private:
    FrameID srcFrame;
    FrameID destFrame;
};

/*! \struct Path
 * \brief Represents a path between two frames. */
struct Path {

    /*! Source frame, frame where the path starts. */
    FrameID src;
    /*! Destination frame, frame where the path ends. */
    FrameID dst;
    /*! Contains all the link of the path from Path::src to Path::dst. The links are ordered in such a way that
     * their ordering represents the path from the source to the destination frame. That is, the first entry in links is the
     * link between the source frame and the first frame along the path towards the destination frame. */
    std::vector< std::reference_wrapper<Link> > links;

    void writeJSON(QJsonObject &json) const;
};

/*! \struct StampedTransformationWithConfidence
 *  \brief Returned as result of a successful query. */
struct StampedTransformationWithConfidence : public StampedTransformation {

    /*! The average \ref Link-Confidence "confidence" of all links used to calculate the transformation. */
    double averageLinkConfidence {0};

    /*! The largest distance between the valid time of transformation entry used for the calculation of the transformation
     * and the query time in the case of a link query or the best time in the case of a best link query.
     * If the default \link TransMem::distanceToEntryMapping mapping\endlink is used, the value is in ms. */
    double maxDistanceToEntry {std::numeric_limits<double>::max()};

};

/*! \class TransMem
 *
 * \section TransMem
 *
 * TransMem is a datastructure which allows to store and retrieve rigid transformations between frames together with a time information.\n
 *
 * \subsection Transformation-Storage Storage of transformation
 * The very first update of a transformation between a source frame and a destination frame creates a <b>link</b> between these two frames and is called <b>registration of a link</b>.
 * The provided transformation, mapping from the source frame to the destination frame, is stored together with a timestamp on the link as a <b>transformation
 * entry</b>. The timestamp indicates at what time the transformation is valid, called <b>valid time</b>. Subsequent updates between these two frames are considered as an <b>update of the link</b>.
 * Every update of a link creates a new transformation entry on the corresponding link. TransMem stores the transformation entries on the links for a specifiable \link TransMem::storageTime time\endlink,
 * transformation entries which are to old are removed.\n\n
 *
 * The storage of transformation can be done with a call to one of the appropriate \link TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans) registerLink(..) \endlink functions.
 *
 * \subsection Transformation-Retrieval Retrieval of transformation
 * There are two possibilites to query TransMem for a transformation.
 *
 * \subsubsection Link-Query Link query
 * One the one hand one can query TransMem for a transformation between a source and a destination frame at a given time, <b>query time</b>. This query is called <b>link query</b>
 * and is done with a call to \link TransMem::getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp) const getLink(..) \endlink.
 * If there exists at least one <b>path</b> connecting the two frames, a <b>transformation</b>, mapping direct from the source to destination frame, can be calculated. The transformation is
 * calculated along of one of the shortest paths, concerning the number of links, by combining the transformation entries of the links building this path. For each transformation entry used
 * in this calculation the following applies:\n
 * - If the query time is smaller than the valid time of the oldest transformation entry stored on the link, TransMem uses the oldest transformation entry for the calculation of the transformation.\n
 * - If the query time is larger than the valid time of the newest transformation entry stored on the link, TransMem uses the newest transformation entry for the calculation of the transformation.\n
 * - If \f$t_1 <\f$ query time \f$< t_2\f$ lies in between two transformation entries stored at valid time \f$t_1\f$,\f$t_2\f$ on a certain link, the library interpolates between the two
 * entries and uses the interpolated mapping for the calculation of the transformation.\n If \f$(t_2-t1) < 5\f$ \f$ns\f$ no interplation is done, the transformation entry with valid time \f$t_1\f$
 * is used.
 *
 * \subsubsection Best-Link-Query Best link query
 * On the other hand one can query TransMem for a <b>best transformation</b> between a source and a destination frame, called <b>best link query</b> and
 * done by using \link TransMem::getBestLink(const FrameID &srcFrame, const FrameID &destFrame) const getBestLink(..)\endlink. Similar to a link query is in a
 * first step the shortest path from the source to the destination frame evaluated. Along this path is then the best transformation calculated.\n\n
 * \anchor Best-Transformation <b>Best transformation:</b>\n
 * Let \f$L_1, .., L_n\f$ be the links of which the shortest path from the source frame to the destination frame consists.
 * Let \f$E_{ij}\f$ be the j-th transformation entry inserted at valid time \f$t_{ij}\f$ in the link \f$L_i\f$.\n
 * The <b>best transformation</b> is then the transformation obtained at time \f$t\f$, were \f$t\f$ minimizes \f$\sum_{i=0}^{n} (t-t_{ij})^2\f$. The \link TransMem::RESOLUTION_BEST_TIME_CALCULATION_IN_MS resolution \endlink of \f$t\f$ is 5 ms.\n
 * Point \f$t\f$ is called <b>best time</b>.\n\n
 *
 * \subsection Locking-behavior Locking behavior
 * TransMem uses a very coarse locking approach, where all public functions have to aquire a single general lock before they are able to acces any data.
 *
 * \subsection Link-Confidence Link confidence
 * The user can assign a confidence value to a link. One can update the confidence of a link direct via the update of this link by
 * specifying a confidence as last argument in the \link TransMem::registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &tstamp, const QMatrix4x4 &trans)
 * registerLink(..)\endlink functions. If no confidence value is specified, then the current confidence of the link remains unchanged. If no confidence is specified in the registration of a link,
 * than the confidence of this link is set to the default link confidence.\n One also can update the confidence of a link without doing a link update with a call to
 * \link TransMem::updateLinkConfidence(const FrameID &srcFrame, const FrameID &destFrame, const double confidence) updateLinkConfidence(..)\endlink.
 * However, a link has to be registered first before such an update can be done.\n\n
 * Note that the link confidence is not considered for the calculation of the shortest path nor for the calculation of the best transformation.
 */
class TransMem
{

friend class GraphMLWriter;

typedef std::function<double(double)> DoubleToDoubleFunction;

public:

    /*! Default constructor.\n
      *
      * Constructs a TransMem object which bufferes the transformation entries for a default duration of 10 seconds.\n
      *
      * The default \ref Link-Confidence "link confidence" is set to 1.
      * The default \link TransMem::distanceToEntryMapping distance to entry mapping\endlink is \f$f(x) = x\f$. */
    TransMem() = default;

    /*! Alternative constructor.\n
     *
     * Constructs a transmem object which bufferes the transformation entries for the duration specified in \a storageTime.
     * If the duration is smaller than one second, the duration is set to one second.
     *
     * \param storageTime Custom value for the buffer duration in seconds.
     * \param defaultLinkConfidence Custom default value for the \ref Link-Confidence "link confidence".
     * \param distanceToEntryMapping Custom mapping for TransMem::distanceToEntryMapping. */
    TransMem(DurationSec storageTime, const double defaultLinkConfidence = 1., const DoubleToDoubleFunction distanceToEntryMapping = [](double x) {return x;})
    : storageTime(storageTime)
    , defaultLinkConfidence(defaultLinkConfidence)
    , distanceToEntryMapping(distanceToEntryMapping)
    {
        if(storageTime < DurationSec(1))
            storageTime = DurationSec(1);
    }

    /*! \fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans)
     *
     * Adds a transformation \a trans mapping from source frame \a srcFrame to the destination frame \a dstFrame valid at time \a valid time
     * as an transformation entry to the link from \a srcFrame to \a dstFrame. If there exists no such link, the link is first created before the
     * transformation entry is added.\n\n
     *
     * In the case of a new creation of a link, the \ref Link-Confidence "confidence" of the newly created link is set to the default value.
     * If there where previous updates of this link, i.e. the link already exists, the confidence of the link does not change.\n\n
     *
     * Does not complain if the newly added link leads to a cycle in the underlying datastructure.\n\n
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \param validTime At what time the transformation is valid.
     * \param trans Describes the transformation mapping from the source to the destination frame. */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans);

    /*! \fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QQuaternion &rotation, const QQuaternion &translation)
     *
     * \see registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans)
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \param validTime At what time the transformation is valid.
     * \param rotation Encodes the rotation part of the transformation mapping from the source to the destination frame.
     * \param translation Encodes the translation part of the transformation mapping from the source to the destination frame. */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QQuaternion &rotation, const QQuaternion &translation);

    /*! \fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans, const double confidence)
     *
     * Adds a transformation \a trans mapping from source frame \a srcFrame to the destination frame \a dstFrame valid at time \a valid time
     * as an transformation entry to the link from \a srcFrame to \a dstFrame. If there exists no such link, the link is first created before the
     * transformation entry is added.\n\n
     *
     * The \ref Link-Confidence "confidence" of the link from \a srcFrame to \a destFrame is set to \a confidence.
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \param validTime At what time the transformation is valid.
     * \param trans Describes the transformation mapping from the source to the destination frame.
     * \param confidence New confidence value of the link between source and destination frame. */
     void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans, const double confidence);

    /*! \fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QQuaternion &rotation, const QQuaternion &translation, const double confidence)
     *
     * \see registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QMatrix4x4 &trans, const double confidence)
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \param validTime At what time the transformation is valid.
     * \param rotation Encodes the rotation part of the transformation mapping from the source frame to the destination frame.
     * \param translation Encodes the translation part of the transformation mapping from the source to the destination frame.
     * \param confidence New confidence value of the link between source and destination frame. */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime, const QQuaternion &rotation, const QQuaternion &tranlation, const double confidence);

    /*! \fn StampedTransformationWithConfidence getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &queryTime) const
     *
     * Returns the transformation mapping from the Frame \a srcFrame to the Frame \a destFrame at query time \a queryTime.
     * For more information check \ref Link-Query "here".
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \param queryTime Timestamp at what time the returned transformation is valid.
     * \return Contains the requested transformation.
     * \throws InvalidArgumentException
     * \throws NoSuchLinkFoundException */
    StampedTransformationWithConfidence getLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &queryTime) const;

    /*! \fn StampedTransformationWithConfidence getBestLink(const FrameID &srcFrame, const FrameID &destFrame) const
     *
     * Returns the best transformation mapping from the frame \a srcFrame to the frame \a destFrame.
     * For more information check \ref Best-Link-Query "here".\n\n
     *
     * The results of the best link queries are cached. A result for a certain best link query is cached until every link needed for the calculation of the transformation
     * was updated or a new link was registered.
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \return Contains the requested best transformation.
     * \throws InvalidArgumentException
     * \throws NoSuchLinkFoundException */
    StampedTransformationWithConfidence getBestLink(const FrameID &srcFrame, const FrameID &destFrame) const;

    /*! \fn void updateLinkConfidence(const FrameID &srcFrame, const FrameID &destFrame, const double confidence)
     *
     * Updates the \ref Link-Confidence "confidence" of the link from source frame \a srcFrame to destination frame \a destFrame with
     * the value of \a confidence. If there was no registration of the link in advance, a NoSuchLinkFoundException is thrown.
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \param confidence New confidence value of the link between the source and the destination frame.
     * \throws NoSuchLinkFoundException */
    void updateLinkConfidence(const FrameID &srcFrame, const FrameID &destFrame, const double confidence);

    /*! \fn dumpAsJSON(QString &path) const
     *
     * Dumps the current state of the TransMem  object as JSON to a file.
     *
     * \param path Were to dump the file. */
    void dumpAsJSON(const QString &path) const;

    /*! \fn void dumpAsGraphML(QString &path) const
     *
     * Dumps the current graph build by the different registered links as GraphML (https://en.wikipedia.org/wiki/GraphML) to a file.
     * The graph represented by the GraphML file can be displayed for example with Gephi. (https://gephi.org/)
     *
     * \param path Were to dump the file. */
    void dumpAsGraphML(const QString &path) const;

protected:

    /*! \fn bool shortestPath(Path &path) const
     *
     * Calculates the shortest path from the source frame \a path.src to the destination frame \a path.dest.
     * The calculation of the shortest path happens with regard to the numbers of links.
     * The \ref Link-Confidence "confidence" of the links is not considered for the calculation of the shortest path.\n\n
     *
     * \param path Specifies source and destination frame. Contains the links forming the shortest path if the function returns true.
     * \return True if there exists a path from source to destination frame. False if there exist no such path. */
    bool shortestPath(Path &path) const;

    /*! \fn void calculateBestPointInTime(Path &path, Timestamp &bestTime) const
     *
     * Calculates the best time as described \ref Best-Transformation "here" for the links building the path \a path.
     *
     * \param path Consists of the links which are considered for the calculation of the best time.
     * \param bestTime Stores best time when the function returns. */
    void calculateBestPointInTime(Path &path, Timestamp &bestTime) const;

    /*! \fn void calculateTransformation(const Path &path, StampedTransformationWithConfidence &resultT) const
     *
     * Calculates the transformation along \a path at the time \a resultT.time.
     *
     * \param path Path along which the transformation is calculated.
     * \param resultT Specifies at what point in time the transformation should be calculated. Contains the
     * transformation when the function returns. */
    void calculateTransformation(const Path &path, StampedTransformationWithConfidence &resultT) const;

    /*! \fn bool bestLink(Path &path, StampedTransformationWithConfidence &stT) const
     *
     * Calculates the best transformation form the source frame \a path.scr to the destination frame \a path.dest.
     * Therefore first the \link TransMem::shortestPath(Path &path) const shortest path \endlink from the source to the destination frame is calculated,
     * then the \link TransMem::calculateBestPointInTime(Path &path, Timestamp &bestPoint) const best time \endlink is evaluated and finally the
     * transformation at best time along the shortest path is \link TransMem::calculateTransformation(const Path &path, StampedTransformationWithConfidence &resultT) const calculated\endlink
     * yielding the best transformation.\n\n
     *
     * \param path Specifies source and destination frame. Contains the shortest path if the method returns true.
     * \param stT Contains the best transformation if the function returns true.
     * \returns True if there exists a path from source frame to destination frame and therefore a best transformation can be calculated.
     * False if there is not such path and therefore no best transformation can be calculated. */
    bool bestLink(Path &path,StampedTransformationWithConfidence &stT) const;


    /*! \fn void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime,
                      const QQuaternion &rotation, const QQuaternion &translation,
                      const double confidence, const bool updateConfidence)
     *
     * Adds the transformation described through the rotation \a rotation and the translation \a translation mapping from source frame \a srcFrame to the
     * destination frame \a dstFrame valid at time \a valid time as an transformation entry to the link from \a srcFrame to \a dstFrame. If there
     * exists no such link, the link is created first before the transformation entry is added.\n\n
     *
     * If the link had to be created first and \a updateConfidence is false then the \ref Link-Confidence "confidence" of the newly created link is set to the default
     * confidence. If \a updateConfidence is true the confidence of the newly created link is set to \a confidence.
     *
     * If there were previous link update, i.e. there was no need to create the link, the confidence of the link is not touched if \a updateConfidence is false
     * and set to \a confidence if \a updateConfidence is true.
     *
     * \param srcFrame Identifier of the source frame.
     * \param destFrame Identifier of the destination frame.
     * \param validTime At what time the transformation is valid.
     * \param rotation Encodes the rotation part of the transformation mapping from the source frame to the destination frame.
     * \param translation Encodes the translation part of the transformation mapping from the source to the destination frame.
     * \param confidence Possible new confidence value of the link between source and destination frame.
     * \param updateConfidence Indicates wether the confidence of the link is updated or not. */
    void registerLink(const FrameID &srcFrame, const FrameID &destFrame, const Timestamp &validTime,
                      const QQuaternion &rotation, const QQuaternion &translation,
                      const double confidence, const bool updateConfidence);

    std::unordered_map<FrameID, Frame> frameID2Frame;

    std::deque<Link> links;

    mutable std::unordered_map< std::string, Path > cachedPaths;
    mutable std::unordered_map< std::string, std::pair< Timestamp, StampedTransformationWithConfidence > > cachedBestTransformations;

    /*! Duration how long transformation entries are stored on a link. */
    DurationSec storageTime {10};

    /*! \see \ref Link-Confidence "Link confidence". */
    const double defaultLinkConfidence  {1.};

    /*! Every result of a link or best link query contains a \link StampedTransformationWithConfidence::maxDistanceToEntry maxDistanceToEntry\endlink value.
     * This value is the largest of all distances to the valid time of the stored transformation entries used for the calculation of the transformation.
     * Per default this value is in ms, i.e. \a distanceToEntryMapping is the identity funcion \f$f(x) = x\f$.
     * Via the constructor of TransMem one can specify a different mapping. For example one can use \f$f(x) = x * 0.001\f$ to get the the value in s. */
    const DoubleToDoubleFunction distanceToEntryMapping = [](double x) {return x;};

    mutable std::recursive_mutex lock;

    /*! Packs the current state of the TransMem object into a JSON object. Needed to dump the current state of a TransMem object to a JSON file.
     * \see  dumpAsJSON(const QString &path) const */
    void writeJSON(QJsonObject &json) const;

    /*! Tolerated deviation at the internal check if provided rotation quaternions and rotation matrices are normal.\n
     * The length of a rotation quaternion should be 1. The absolute value of a rotation matrix should also be 1. There is no warning emitted if the values differ
     * at most \a TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK from 1. */
    const double TOLERATED_DEVIATION_ROTATION_NORMAL_CHECK {0.005};

    /*! \see \ref Best-Link-Query "Best link query" */
    const int RESOLUTION_BEST_TIME_CALCULATION_IN_MS {5};

};

#endif // TRANSMEM_H
