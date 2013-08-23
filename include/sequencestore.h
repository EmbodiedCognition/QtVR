#ifndef SEQUENCESTORE_H
#define SEQUENCESTORE_H

#include <QObject>
#include <QString>

#include "DataCValues.h"
#include "CapBody.h"

struct MarkerFrame
{
  double data[LED_COUNT][4];
};

class MarkerSequence
{
  QList<MarkerFrame> data;

  MarkerSequence(QString filename);

  /// Add a frame to the sequence
  void appendFrame(const MarkerFrame& frame);

  /// Get a frame from the sequence
  const MarkerFrame* getFrame(int frameNo);

  /// Create a frame on the end of the sequence
  /// that can then be modified/filled in.
  MarkerFrame* getNewFrame();

};

/**
  Either torques or angles (position)
  */
struct JointFrame
{
  double data[CapBody::JOINT_COUNT][3];
};

class JointSequence
{
  QList<JointFrame> data;


};

/**
  A sequence storage.
  */
class SequenceStore : public QObject
{
    Q_OBJECT
public:
    explicit SequenceStore(QObject *parent = 0);

  // Get list of sequences: string label, id
  // Create a new sequence
  // Save frame to sequence
  // Load sequence from file
  // Offload sequence
  // Save sequence to file
  // Change sequence offset
  // Get sequence data


signals:

public slots:

protected:
  QList<MarkerSequence> markerSequences;
  QList<JointSequence> angleSequences;
  QList<JointSequence> torqueSequences;

};

#endif // SEQUENCESTORE_H
