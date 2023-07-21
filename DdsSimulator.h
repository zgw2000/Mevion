/**********************************************************************************
** THIS SOFTWARE IS THE SOLE PROPERTY OF MEVION MEDICAL SYSTEMS, INC.            **
** ANY USE, REPRODUCTION, OR DISTRIBUTIONS AS A WHOLE OR IN PART WITHOUT THE     **
** WRITTEN PERMISSION OF MEVION MEDICAL SYSTEMS, INC. IS PROHIBITED. THIS NOTICE **
** MAY NOT BE REMOVED WITHOUT WRITTEN APPROVAL FROM MEVION MEDICAL SYSTEMS, INC. **
**                                                                               **
** COPYRIGHT (c) 2016                                                            **
** BY MEVION MEDICAL SYSTEMS, INC., LITTLETON, MA, USA.                          **
**********************************************************************************/

#include "ui_DdsSimulator.h"
#include <QMainWindow>
#include <QTimer>
#include <memory>

#include "state_topics.h"
#include "PlanBeam.h"
#include "PatientPlanInfo.h"
#include "BeamInfo.h"
#include "CouchDelta.h"
#include "CouchCig.h"
#include "CouchCigSetpoint.h"
#include "CouchGeometry.h"
#include "TableTopGeometry.h"
#include "RoomGeometry.h"
#include "HandPendantCmds.h"
#include "state_topics.h"
#include "SessionReset.h"
#include "MevJsonConfigFile.h"
#include "ScannerPositionConfig_v2.h"
#include "ScannerCalibrationConfig_v2.h"
#include "ScannerCalibration_v2.h"
#include "AppStatus.h"

#include "MevDDS_ListenerQtSignal_T.hpp"
#include "MevDDS_WriterQtSignal_T.hpp"

// DDS topics for listening
Q_DECLARE_METATYPE(PlanBeam)
Q_DECLARE_METATYPE(srsdds::CouchDelta)

// DDS topics for publishing
Q_DECLARE_METATYPE(AppStatus)
Q_DECLARE_METATYPE(srsdds::CouchCig)
Q_DECLARE_METATYPE(srsdds::CouchCigSetpoint)
Q_DECLARE_METATYPE(srsdds::HandPendantCmds)
Q_DECLARE_METATYPE(std::shared_ptr<AppStatus>)
Q_DECLARE_METATYPE(std::shared_ptr<srsdds::CouchCig>)
Q_DECLARE_METATYPE(std::shared_ptr<srsdds::CouchCigSetpoint>)
Q_DECLARE_METATYPE(std::shared_ptr<srsdds::HandPendantCmds>)
Q_DECLARE_METATYPE(srsdds::CouchDeltaLocation)
Q_DECLARE_METATYPE(srsdds::CouchIsocenter)
Q_DECLARE_METATYPE(srsdds::SessionReset)

class QSerialPort;

typedef struct {double x; double y; double z; double p; double r; double rot; } point_6d;

static const double minX = -500.0;
static const double maxX = 500.0;
static const double minY = -500.0;
static const double maxY = 500.0;
static const double minZ = -93.0;
static const double maxZ = 50.0;
static const double minRot = 45.0;
static const double maxRot = 135.0;
static const double minPitch = -5.0;
static const double maxPitch = 5.0;
static const double minRoll = -5.0;
static const double maxRoll = 5.0;
static const double minExt = 0.0;
static const double maxExt = 33.6;
static const double minGantry = -5.0;
static const double maxGantry = 185.0;

static const mev::RoomGeometry load_pos(-43.0, 142.0, -36.0, 0.0, 0.0, 330.0);

class DdsSimulator : public QMainWindow, private Ui::MainWindow
{
    Q_OBJECT
    
public:
    DdsSimulator(int domainID, bool enableSerialInput = false, QWidget *parent = nullptr);
    
protected:
    srsdds::CouchIsocenter updateAndPublishIsocenter();

private slots:
    void on_prepForMotion_toggled(bool);
    void setGantryAngleType();
    void setGantry90Type();
    void setGantry180Type();
    void setGantryExtendType();
    
    void setCouchSetupType();
    void setCouchTxType();
    void setCouchDeltaType();
    
    void setCouchLoadType();
    void setCouchSaveType();
    void setCouchRecallType();
    void setCouchSwitchType();
    void timerUpdate();
    void movePressed();
    void moveReleased();
    void jogPlusGAPressed();
    void jogPlusGAReleased();
    void jogMinusGAPressed();
    void jogMinusGAReleased();
    
    void jogPlusGExtPressed();
    void jogPlusGExtReleased();
    void jogMinusGExtPressed();
    void jogMinusGExtReleased();
    
    void jogPlusXPressed();
    void jogPlusXReleased();
    void jogMinusXPressed();
    void jogMinusXReleased();
    void zeroXPressed();
    
    void jogPlusYPressed();
    void jogPlusYReleased();
    void jogMinusYPressed();
    void jogMinusYReleased();
    void zeroYPressed();

    void jogPlusZPressed();
    void jogPlusZReleased();
    void jogMinusZPressed();
    void jogMinusZReleased();
    void zeroZPressed();

    void jogPlusRotPressed();
    void jogPlusRotReleased();
    void jogMinusRotPressed();
    void jogMinusRotReleased();
    
    void jogPlusPitchPressed();
    void jogPlusPitchReleased();
    void jogMinusPitchPressed();
    void jogMinusPitchReleased();
    
    void jogPlusRollPressed();
    void jogPlusRollReleased();
    void jogMinusRollPressed();
    void jogMinusRollReleased();

    void onPlanBeamReceived();
    void onCouchCigSetpointReceived();
    void onCouchDeltasReceived();
    
    void xrayButtonClicked();
    void ticUpdate();
    void beamPrepClicked();
    void couchStateChanged(int );
    
    void motionEnablePressed();
    void EStopPressed();
    
    void readSerialData();
    void sendSerialData();

    void resetSession();
    void publishCapturedCouchLocation();
    
    void onPmcCheckboxClicked();

    void onSendScanPositionClicked();

private:
    enum CouchMode { eSetup, eTx, eDelta };
    enum CouchCmd { eNoCmd, eGotoSetup, eGotoTx, eGotoDelta, eAngle, e90, e180, eExtend, eLoad, eSave, eRecall, eSwitch };
    enum buttonPresses { eNone, eMove, ePlusGA, eMinusGA, ePlusGExt, eMinusGExt, ePlusX, eMinusX,
		      ePlusY, eMinusY, ePlusZ, eMinusZ, ePlusRot, eMinusRot, ePlusPitch, eMinusPitch,
        ePlusRoll, eMinusRoll, eXrayInOut, eBeamPrep };
    
    CouchMode m_mode;
    CouchCmd cmd;
    bool   m_preppedForMotion;
    double m_actualGantryPos;
    double m_actualGantryExt;
    double m_planGantryPos;
    double m_planGantryExt;

    point_6d m_planSetupPos;
    point_6d m_planIsoPos;
    point_6d m_planDeltaPos;

    mev::RoomGeometry m_savePos;
    mev::RoomGeometry m_actualRoomPos;
    mev::RoomGeometry m_currentDeltaSet;

    mev::TableTopGeometry m_ttIsocenter;

    CouchMode m_saveMode;
    bool m_isDeltaValid;
    bool m_isSetupValid;
    bool m_isTxValid;
    bool m_isValidPlanData;
    QTimer *m_timer;
    QTimer *m_ticTimer;
    long m_timetic;
    buttonPresses m_activeButton;
    
    bool loadCouchIoConfig();
    void errorInitializingDDSlistener( DDS_Topic_ID topicId );
    void initData();
    void deselectGantryCmds();
    void deselectCouchCmds();
    void update();
    void setSetupLabels();
    void setTxLabels();
    void closeEvent(QCloseEvent *event);
    void retractApplicator();
    void advanceCouchTxMode(point_6d dest);
    void advanceCouchSetupMode(point_6d dest);
    void advanceCouchDeltaMode(void);
    void init_setupData();
    bool isPointEqual(point_6d src, point_6d dest);
    void emitCouchSetpoint(srsdds::PresetID preset, const point_6d *p);
    void emitCouchSetpoint1(srsdds::PresetID preset, const double p);
    void keyPressEvent(QKeyEvent *e);
    
    void processButton(int byteIndex,
                       char mask,
                       void (DdsSimulator::*pressFP)(),
                       void (DdsSimulator::*releaseFP)());
    
    void initializeSerialPort();

    //Helper functions to perform actual movement. Will adjust what is moved based on mode.
    //e.g. X will be adjusted when in Setup, Lateral will be adjusted when in Treatment.
    void moveGantryAngle(double step);
    void moveGantryExtension(double step);
    void moveXorLateral(double step);
    void moveYorLongitudinal(double step);
    void moveZ(double step);
    void moveRotation(double step);
    void movePitch(double step);
    void moveRoll(double step);

    DDS_Utility m_ddsUtil;

    std::unique_ptr<MevJsonConfigFile> m_couchIoConfig;
    std::unique_ptr<mev::ScannerPositionConfig_v2> m_scanPosConfig;
    std::unique_ptr<mev::ScannerCalibrationConfig_v2> m_scanCalConfig;

    std::map<srsdds::ScannerId, mev::ScannerCalibration_v2> m_scannerCalMap;

    srsdds::CouchCigSetpoint m_couchPresetLocationInstance;
    srsdds::CouchCigSetpoint m_gantryInstance;
    srsdds::CouchCigSetpoint m_extensionInstance;
    srsdds::HandPendantCmds  m_handPendantCmd;
    
    AppStatus m_couchIOStatus;
    
    // Components for reading pendant data from the serial port
    bool         m_serialPortInitialized;
    QSerialPort* m_serialPortPtr;
    char         m_serialReadBuffer[256];
    char         m_serialDataBytes[6];
    char         m_internalDataBytes[6];
    QByteArray   m_readDataBytes;
    
    typedef MevDDS_ListenerQtSignal<PlanBeam,
                                    PlanBeamSeq,
                                    PlanBeamDataReader>
                                    PlanBeamListenerType;
        
    typedef MevDDS_ListenerQtSignal<srsdds::CouchCigSetpoint,
                                    srsdds::CouchCigSetpointSeq,
                                    srsdds::CouchCigSetpointDataReader>
                                    CouchCigSetpointListenerType;

    typedef MevDDS_ListenerQtSignal<srsdds::CouchDelta,
                                    srsdds::CouchDeltaSeq,
                                    srsdds::CouchDeltaDataReader>
                                    CouchDeltaListenerType;

    typedef MevDDS_ListenerQtSignal<srsdds::SessionReset,
                                    srsdds::SessionResetSeq,
                                    srsdds::SessionResetDataReader>
                                    SessionResetListenerType;

    typedef MevDDS_WriterQtSignal<AppStatus,
                                  AppStatusDataWriter>
                                  AppStatusWriterType;

    typedef MevDDS_WriterQtSignal<srsdds::CouchCig,
                                  srsdds::CouchCigDataWriter>
                                  CouchCigWriterType;

    typedef MevDDS_WriterQtSignal<srsdds::CouchCigSetpoint,
                                  srsdds::CouchCigSetpointDataWriter>
                                  CouchCigSetpointWriterType;

    typedef MevDDS_WriterQtSignal<srsdds::HandPendantCmds,
                                  srsdds::HandPendantCmdsDataWriter>
                                  HandPendantCmdsWriterType;

    typedef MevDDS_WriterQtSignal<srsdds::CouchDeltaLocation,
                                  srsdds::CouchDeltaLocationDataWriter>
                                  CouchDeltaLocationWriterType;

    typedef MevDDS_WriterQtSignal<srsdds::CouchIsocenter,
                                  srsdds::CouchIsocenterDataWriter>
                                  CouchIsocenterWriterType;

    typedef MevDDS_ListenerQtSignal<srsdds::CouchDeltaLocationRequest,
                                    srsdds::CouchDeltaLocationRequestSeq,
                                    srsdds::CouchDeltaLocationRequestDataReader>
                                    CouchDeltaLocRqstListenerType;

    typedef MevDDS_WriterQtSignal<srsdds::CouchScannerResult,
                                  srsdds::CouchScannerResultDataWriter>
                                  CouchScannerResultWriterType;

    CouchCigSetpointListenerType*   m_couchCigSetpointListener;
    CouchDeltaListenerType*         m_couchDeltaListener;
    PlanBeamListenerType*           m_planBeamListener;
    SessionResetListenerType*       m_sessionResetListener;
    AppStatusWriterType*            m_appStatusPub;
    CouchCigWriterType*             m_couchCigPub;
    CouchCigSetpointWriterType*     m_couchCigSetpointPub;
    HandPendantCmdsWriterType*      m_handPendantCmdsPub;
    CouchDeltaLocationWriterType*   m_couchDeltaLocationPub;
    CouchDeltaLocationWriterType*   m_CouchCapturedLocationPub;
    CouchIsocenterWriterType*       m_couchIsocenterPub;
    CouchDeltaLocRqstListenerType*  m_CouchDeltaLocRqstListener;
    CouchScannerResultWriterType*   m_couchScannerResultPub;
};
