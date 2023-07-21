/***********************************************************************************
 ** THIS SOFTWARE IS THE SOLE PROPERTY OF MEVION MEDICAL SYSTEMS, INC.            **
 ** ANY USE, REPRODUCTION, OR DISTRIBUTIONS AS A WHOLE OR IN PART WITHOUT THE     **
 ** WRITTEN PERMISSION OF MEVION MEDICAL SYSTEMS, INC. IS PROHIBITED. THIS NOTICE **
 ** MAY NOT BE REMOVED WITHOUT WRITTEN APPROVAL FROM MEVION MEDICAL SYSTEMS, INC. **
 **                                                                               **
 ** COPYRIGHT (c) 2016                                                            **
 ** BY MEVION MEDICAL SYSTEMS, INC., LITTLETON, MA, USA.                          **
 **********************************************************************************/

#include "DdsSimulator.h"
#include <QKeyEvent>
#include "MevMathUtil.h"
#include "MevCompare.h"
#include "DDS_Common.h"
#include "CouchGeometry.h"
#include "version_number.h"
#include "MevDdsUtil.h"
#include "MevBit.h"
#include "CouchConfigDataPlugin.h"

// IDL Topics
#include "state_topicsSupport.h"
#include "PlanBeamSupport.h"
#include "CouchDeltaSupport.h"
#include "CouchCigSupport.h"
#include "CouchCigSetpointSupport.h"
#include "HandPendantCmdsSupport.h"
#include "SessionResetSupport.h"
#include "AppStatusSupport.h"

// Include support for reading pendant command directly via serial
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <QDebug>
#include <QThread>

Q_DECLARE_METATYPE(srsdds::ScannerId)

namespace
{
    const char* json_ScannerPositionConfig    = "ScannerPositionConfig";
    const char* json_ScannerCalibrationConfig = "ScannerCalibrationConfig";

    const unsigned short NO_CALLBACK_FUNCTION             = 0;
    const unsigned short SERIAL_READ_BUFFER_SIZE_IN_BYTES = 256;
    const unsigned short LENGTH_OF_HEADER_STRING          = 19;
    const unsigned short PENDANT_SEND_TIMEOUT_IN_MS       = 30000;

    /**
     * @brief Conversion factor for converting centimeters to millimeters
     */
    const float CM_TO_MM_CONVERSION_FACTOR = 10.0;

    /** Utility function for converting between RoomGeometry and the internal point_6d type. */
    point_6d RoomGeoToPoint6D(const mev::RoomGeometry& roomGeo)
    {
        point_6d retVal;
        retVal.x = roomGeo.x_mm();
        retVal.y = roomGeo.y_mm();
        retVal.z = roomGeo.z_mm();
        retVal.rot = roomGeo.tableAngle_deg();
        retVal.p = roomGeo.pitch_deg();
        retVal.r = roomGeo.roll_deg();

        return retVal;
    }
}

/**
 * @var HandPendant::m_savePos
 * @brief Position stored in the Save preset. Room coordinates. mm.
 */

/**
 * @var HandPendant::m_actualRoomPos
 * @brief Current position of the couch. Room coordinates. mm.
 */

/**
 * @var HandPendant::m_currentDeltaSet
 * @brief Remanining deltas stored in the delta preset. Room coordinates. mm.
 *
 * As the couch moves towards the preset, these will go to zero as m_actualRoomPos approaces the
 * target location.
 */

/**
 * @var HandPendant::m_ttIsocenter
 * @brief Current isocenter of the couch. IEC TableTop coordinates. mm.
 */

/**
 * @brief Constructor
 * @param domainID DDS domain identifier
 * @param enableSerialInput If 'true', hand pendant commands will be read from the serial port.
 * @param parent Pointer to the parent widget
 */
DdsSimulator::DdsSimulator (int domainID, bool enableSerialInput, QWidget *parent)
: QMainWindow(parent),
  m_ddsUtil( domainID, DDS_Utility::BASE_QOS_PROFILE ),
  m_serialPortInitialized(false),
  m_serialPortPtr(0),
  m_CouchDeltaLocRqstListener(nullptr)
{
    setupUi(this);

    // Register DDS shared pointers needed for publishing
    qRegisterMetaType< std::shared_ptr< AppStatus > >(
                                                             "std::shared_ptr<AppStatus>"
                                                             );
    
    qRegisterMetaType< std::shared_ptr< srsdds::CouchCig > >(
                                                             "std::shared_ptr<srsdds::CouchCig>"
                                                             );
    
    qRegisterMetaType< std::shared_ptr< srsdds::CouchCigSetpoint > >(
                                                    "std::shared_ptr<srsdds::CouchCigSetpoint>"
                                                        );

    qRegisterMetaType< std::shared_ptr< srsdds::HandPendantCmds > >(
                                                "std::shared_ptr<srsdds::HandPendantCmds>"
                                                             );
    
    
    // Initialize MevDDS
    DDS_Common::initialize();

    initData();
    
    // Create publishers
    DDS_Utility& ddsUtil = m_ddsUtil;
    
    m_appStatusPub = new AppStatusWriterType(ddsUtil);
    m_appStatusPub->initialize(MCC_TC_AppStatusSet);
    
    m_couchCigPub = new CouchCigWriterType(ddsUtil);
    m_couchCigPub->initialize(TDS_CouchCig);
    
    m_couchCigSetpointPub = new CouchCigSetpointWriterType(ddsUtil);
    m_couchCigSetpointPub->initialize(TDS_CouchCigSetpointGet);

    m_handPendantCmdsPub = new HandPendantCmdsWriterType(ddsUtil);
    m_handPendantCmdsPub->initialize(TDS_HandPendantCmds);

    m_couchDeltaLocationPub = new CouchDeltaLocationWriterType(ddsUtil);
    m_couchDeltaLocationPub->initialize(TDS_CouchDeltaLocationFieldComplete);

    m_CouchCapturedLocationPub = new CouchDeltaLocationWriterType(ddsUtil);
    m_CouchCapturedLocationPub->initialize(TDS_CouchDeltaLocationCouchCapture);

    m_couchIsocenterPub = new CouchIsocenterWriterType(ddsUtil);
    m_couchIsocenterPub->initialize(TDS_CouchIsocenter);

    m_couchScannerResultPub = new CouchScannerResultWriterType(ddsUtil);
    m_couchScannerResultPub->initialize(Verity_CouchScannerResult);

    m_planBeamListener = new PlanBeamListenerType(ddsUtil);
    bool res = m_planBeamListener->initialize(TDS_PlanBeam);
    if(!res)
    {
        errorInitializingDDSlistener(m_planBeamListener->getTopicId());
    }
    MEV_ASSERT(connect(m_planBeamListener,
                       SIGNAL(ddsOnDataAvailable()),
                       this,
                       SLOT(onPlanBeamReceived())));
    
    m_couchCigSetpointListener = new CouchCigSetpointListenerType(ddsUtil);
    res = m_couchCigSetpointListener->initialize(TDS_CouchCigSetpoint);
    if(!res)
    {
        errorInitializingDDSlistener(m_couchCigSetpointListener->getTopicId());
    }
    MEV_ASSERT(connect(m_couchCigSetpointListener,
                       SIGNAL(ddsOnDataAvailable()),
                       this,
                       SLOT(onCouchCigSetpointReceived())));


    m_couchDeltaListener = new CouchDeltaListenerType(ddsUtil);
    res = m_couchDeltaListener->initialize(TDS_CouchDelta);
    if(!res)
    {
        errorInitializingDDSlistener(m_couchDeltaListener->getTopicId());
    }
    MEV_ASSERT(connect(m_couchDeltaListener,
                       SIGNAL(ddsOnDataAvailable()),
                       this,
                       SLOT(onCouchDeltasReceived())));
    
    m_sessionResetListener = new SessionResetListenerType(ddsUtil);
    if ( !m_sessionResetListener->initialize(TDS_SessionReset) )
    {
        errorInitializingDDSlistener(m_sessionResetListener->getTopicId());
    }
    MEV_ASSERT(connect(m_sessionResetListener, &SessionResetListenerType::ddsOnDataAvailable,
                       this, &DdsSimulator::resetSession));

    m_CouchDeltaLocRqstListener = new CouchDeltaLocRqstListenerType(ddsUtil);
    res = m_CouchDeltaLocRqstListener->initialize(TDS_CouchDeltaLocationRequest);
    if(!res)
    {
        std::string topicBase;
        std::string topicName;
        MEV_ASSERT(DDS_Common::getTopicName(m_CouchDeltaLocRqstListener->getTopicId(), topicBase, topicName));
        const QString msg = QString("Failed to init DDS listener for topic (%1::%2).")
                                    .arg(topicBase.c_str())
                                    .arg(topicName.c_str());
        MEV_LOG_ERROR(mev::CATEGORY_DDS_INITIALIZATION_ERROR,
                      mev::SEVERITY_FATAL,
                      msg.toStdString());
    }
    MEV_ASSERT(connect(m_CouchDeltaLocRqstListener,
                       &CouchDeltaLocRqstListenerType::ddsOnDataAvailable,
                       this, &DdsSimulator::publishCapturedCouchLocation));

    m_couchIOStatus.appID = PTS250_DEVICE_COUCH_IO;
    m_couchIOStatus.appStatusBits = 0;
    
    MEV_ASSERT(connect(gantrySelect, SIGNAL(clicked()), this, SLOT(setGantryAngleType())));
    MEV_ASSERT(connect(gantry90Select, SIGNAL(clicked()), this, SLOT(setGantry90Type())));
    MEV_ASSERT(connect(gantry180Select, SIGNAL(clicked()), this, SLOT(setGantry180Type())));
    MEV_ASSERT(connect(gExtendSelect, SIGNAL(clicked()), this, SLOT(setGantryExtendType())));
    
    MEV_ASSERT(connect(couchSetupSelect, SIGNAL(clicked()), this, SLOT(setCouchSetupType())));
    MEV_ASSERT(connect(couchTxSelect, SIGNAL(clicked()), this, SLOT(setCouchTxType())));
    MEV_ASSERT(connect(couchDeltaSelect, SIGNAL(clicked()), this, SLOT(setCouchDeltaType())));
    
    MEV_ASSERT(connect(couchLoadSelect, SIGNAL(clicked()), this, SLOT(setCouchLoadType())));
    MEV_ASSERT(connect(couchSaveSelect, SIGNAL(clicked()), this, SLOT(setCouchSaveType())));
    MEV_ASSERT(connect(couchRecallSelect, SIGNAL(clicked()), this, SLOT(setCouchRecallType())));
    MEV_ASSERT(connect(couchSwitchSelect, SIGNAL(clicked()), this, SLOT(setCouchSwitchType())));
    
    MEV_ASSERT(connect(moveButton, SIGNAL(pressed()), this, SLOT(movePressed())));
    MEV_ASSERT(connect(moveButton, SIGNAL(released()), this, SLOT(moveReleased())));
    MEV_ASSERT(connect(m_timer, SIGNAL(timeout()), this, SLOT(timerUpdate())));
    MEV_ASSERT(connect(m_ticTimer, SIGNAL(timeout()), this, SLOT(ticUpdate())));
    
    MEV_ASSERT(connect(jogPlusGA, SIGNAL(pressed()), this, SLOT(jogPlusGAPressed())));
    MEV_ASSERT(connect(jogPlusGA, SIGNAL(released()), this, SLOT(jogPlusGAReleased())));
    MEV_ASSERT(connect(jogMinusGA, SIGNAL(pressed()), this, SLOT(jogMinusGAPressed())));
    MEV_ASSERT(connect(jogMinusGA, SIGNAL(released()), this, SLOT(jogMinusGAReleased())));
    
    MEV_ASSERT(connect(jogPlusGExt, SIGNAL(pressed()), this, SLOT(jogPlusGExtPressed())));
    MEV_ASSERT(connect(jogPlusGExt, SIGNAL(released()), this, SLOT(jogPlusGExtReleased())));
    MEV_ASSERT(connect(jogMinusGExt, SIGNAL(pressed()), this, SLOT(jogMinusGExtPressed())));
    MEV_ASSERT(connect(jogMinusGExt, SIGNAL(released()), this, SLOT(jogMinusGExtReleased())));
    
    MEV_ASSERT(connect(jogPlusX, SIGNAL(pressed()), this, SLOT(jogPlusXPressed())));
    MEV_ASSERT(connect(jogPlusX, SIGNAL(released()), this, SLOT(jogPlusXReleased())));
    MEV_ASSERT(connect(jogMinusX, SIGNAL(pressed()), this, SLOT(jogMinusXPressed())));
    MEV_ASSERT(connect(jogMinusX, SIGNAL(released()), this, SLOT(jogMinusXReleased())));
    MEV_ASSERT(connect(zeroX,     SIGNAL(pressed()), this, SLOT(zeroXPressed())));

    MEV_ASSERT(connect(jogPlusY, SIGNAL(pressed()), this, SLOT(jogPlusYPressed())));
    MEV_ASSERT(connect(jogPlusY, SIGNAL(released()), this, SLOT(jogPlusYReleased())));
    MEV_ASSERT(connect(jogMinusY, SIGNAL(pressed()), this, SLOT(jogMinusYPressed())));
    MEV_ASSERT(connect(jogMinusY, SIGNAL(released()), this, SLOT(jogMinusYReleased())));
    MEV_ASSERT(connect(zeroY,     SIGNAL(pressed()), this, SLOT(zeroYPressed())));

    MEV_ASSERT(connect(jogPlusZ, SIGNAL(pressed()), this, SLOT(jogPlusZPressed())));
    MEV_ASSERT(connect(jogPlusZ, SIGNAL(released()), this, SLOT(jogPlusZReleased())));
    MEV_ASSERT(connect(jogMinusZ, SIGNAL(pressed()), this, SLOT(jogMinusZPressed())));
    MEV_ASSERT(connect(jogMinusZ, SIGNAL(released()), this, SLOT(jogMinusZReleased())));
    MEV_ASSERT(connect(zeroZ,     SIGNAL(pressed()), this, SLOT(zeroZPressed())));

    MEV_ASSERT(connect(jogPlusRot, SIGNAL(pressed()), this, SLOT(jogPlusRotPressed())));
    MEV_ASSERT(connect(jogPlusRot, SIGNAL(released()), this, SLOT(jogPlusRotReleased())));
    MEV_ASSERT(connect(jogMinusRot, SIGNAL(pressed()), this, SLOT(jogMinusRotPressed())));
    MEV_ASSERT(connect(jogMinusRot, SIGNAL(released()), this, SLOT(jogMinusRotReleased())));
    
    MEV_ASSERT(connect(jogPlusPitch, SIGNAL(pressed()), this, SLOT(jogPlusPitchPressed())));
    MEV_ASSERT(connect(jogPlusPitch, SIGNAL(released()), this, SLOT(jogPlusPitchReleased())));
    MEV_ASSERT(connect(jogMinusPitch, SIGNAL(pressed()), this, SLOT(jogMinusPitchPressed())));
    MEV_ASSERT(connect(jogMinusPitch, SIGNAL(released()), this, SLOT(jogMinusPitchReleased())));
    
    MEV_ASSERT(connect(jogPlusRoll, SIGNAL(pressed()), this, SLOT(jogPlusRollPressed())));
    MEV_ASSERT(connect(jogPlusRoll, SIGNAL(released()), this, SLOT(jogPlusRollReleased())));
    MEV_ASSERT(connect(jogMinusRoll, SIGNAL(pressed()), this, SLOT(jogMinusRollPressed())));
    MEV_ASSERT(connect(jogMinusRoll, SIGNAL(released()), this, SLOT(jogMinusRollReleased())));
    
    MEV_ASSERT(connect(beamPrepButton, SIGNAL(clicked()), this, SLOT(beamPrepClicked())));
    MEV_ASSERT(connect(xrayButton, SIGNAL(clicked()), this, SLOT(xrayButtonClicked())));
    MEV_ASSERT(connect(couchInfo,
                       SIGNAL(currentIndexChanged (int)),
                       this,
                       SLOT(couchStateChanged(int))));

    MEV_ASSERT(connect(pmcSelect,
                       &QCheckBox::clicked,
                       this,
                       &DdsSimulator::onPmcCheckboxClicked));

    //setWindowFlags(Qt::FramelessWindowHint);
    
    versionLabel->setText(tr("v%1.%2.%3")
                          .arg(MEV_MAJOR_VERSION)
                          .arg(MEV_MINOR_VERSION)
                          .arg(MEV_MICRO_VERSION));

    ::memset(m_serialReadBuffer, 0, SERIAL_READ_BUFFER_SIZE_IN_BYTES);

    if (enableSerialInput)
    {
        initializeSerialPort();

        // Register a callback to process new serial data as it is received
        MEV_ASSERT(connect(m_serialPortPtr, SIGNAL(readyRead()), this, SLOT(readSerialData())));

        // Send a couple of stop commands to the hub to make sure it isn't trying to
        // communicate with the motor controller. We will also send this any time
        // the Done/Beam Prep button is pressed.
        sendSerialData();
        sendSerialData();
    }

    locationTableWidget->setVisible(false);

    // Try to load CouchIO config
    if(!loadCouchIoConfig())
    {
        scannerComboBox->setEnabled(false);
        sendScanPosButton->setEnabled(false);
        atScanPosCheckBox->setEnabled(false);
    }
}

/**
 * @brief Load and process the CouchIO config file.
 * @return true on success, false otherwise.
 */
bool DdsSimulator::loadCouchIoConfig()
{
    // Load file
    m_couchIoConfig.reset(new MevJsonConfigFile(
                                    CouchConfigDataPlugin::getConfigFilename().toStdString()));
    if(m_couchIoConfig->loadFile() != MevConfigFile::MCF_OK)
    {
        m_couchIoConfig.reset();
        return false;
    }

    // Load scan position config
    {
        m_scanPosConfig.reset(new mev::ScannerPositionConfig_v2);

        // Get CT_ScannerPositionConfig object
        const mev::JsonValue& js_scanPosConfig = m_couchIoConfig->value(json_ScannerPositionConfig);
        if(js_scanPosConfig.isNull() || !js_scanPosConfig.isObject())
        {
            const QString msg = QString("Invalid (%1) in config file (%2).")
                                        .arg(json_ScannerPositionConfig)
                                        .arg(m_couchIoConfig->filename().c_str());
            MEV_LOG_ERROR(mev::CATEGORY_CONFIG_ERROR, mev::SEVERITY_FATAL, msg.toStdString());
        }

        // Load scan positions
        if(!m_scanPosConfig->fromJson(js_scanPosConfig.objectVal()))
        {
            const QString msg = QString("Failed to load one or more CT scanner positions.");
            MEV_LOG_ERROR(mev::CATEGORY_CONFIG_ERROR, mev::SEVERITY_FATAL, msg.toStdString());

            return false;
        }
    }

    // Load scanner calibrations
    {
        m_scanCalConfig.reset(new mev::ScannerCalibrationConfig_v2);

        // Get calibration set object
        const mev::JsonValue& js_ScannerCalibration = m_couchIoConfig->value(json_ScannerCalibrationConfig);
        if(js_ScannerCalibration.isNull() || !js_ScannerCalibration.isObject())
        {
            QString errMsg = QString("Invalid or missing setting (%1) in config file (%2).")
                                    .arg(json_ScannerCalibrationConfig)
                                    .arg(m_couchIoConfig->filename().c_str());
            MEV_LOG_ERROR(mev::CATEGORY_PARSE_ERROR, mev::SEVERITY_CRITICAL, errMsg.toStdString());

            return false;
        }

        // Load object
        if(!m_scanCalConfig->fromJson(js_ScannerCalibration.objectVal()))
        {
            QString errMsg = QString("Invalid (%1) in config file (%2).")
                                    .arg(json_ScannerCalibrationConfig)
                                    .arg(m_couchIoConfig->filename().c_str());
            MEV_LOG_ERROR(mev::CATEGORY_PARSE_ERROR, mev::SEVERITY_CRITICAL, errMsg.toStdString());

            return false;
        }
    }

    // Add scanner to combo
    scannerComboBox->clear();
    const std::vector<srsdds::ScannerId> scannerIds = m_scanPosConfig->scannerIds();
    for(const srsdds::ScannerId scannerId : scannerIds)
    {
        // Get scanner name
        const char* scannerName = getString_ScannerId(scannerId);

        // Add to combo
        scannerComboBox->addItem(scannerName, QVariant::fromValue(scannerId));
    }

    MEV_ASSERT(connect(sendScanPosButton,
                       &QPushButton::clicked,
                       this,
                       &DdsSimulator::onSendScanPositionClicked));

    // Success
    return true;
}

/**
 * @brief An error occurred while attempting to initialize a DDS listener.
 * @details Log the error results using the Mevion DDS log handler.
 * @param topicId RTI DDS Topic Identifier
 */
void DdsSimulator::errorInitializingDDSlistener( DDS_Topic_ID topicId )
{
    std::string topicBase;
    std::string topicName;
    MEV_ASSERT(DDS_Common::getTopicName( topicId, topicBase, topicName));
    const QString msg = QString("Failed to init DDS listener for topic (%1::%2).")
                                .arg(topicBase.c_str())
                                .arg(topicName.c_str());
    MEV_LOG_ERROR(mev::CATEGORY_DDS_INITIALIZATION_ERROR,
                  mev::SEVERITY_CRITICAL,
                  msg.toStdString());
}

/**
 * @brief Initialize all data to default values.
 * @details Called by the Constructor
 */
void DdsSimulator::initData()
{
    m_mode = eSetup;

    couchSetupSelect->setChecked(true);
    deselectGantryCmds();
    deselectCouchCmds();
    setSetupLabels();
    cmd = eGotoSetup;
    couchDeltaSelect->setCheckable(false);
    gantrySelect->setCheckable(false);
    gExtendSelect->setCheckable(false);
    m_timer = new QTimer(this);
    m_ticTimer = new QTimer(this);
    m_ticTimer->start(300);

    m_isDeltaValid = false;
    m_isSetupValid = false;
    m_isTxValid = false;
    m_activeButton = eNone;
    m_actualGantryPos = 90;
    m_actualGantryExt = 33.6;
    m_planGantryPos = 0.0;
    m_planGantryExt = 0.0;
    m_timetic=0;
    m_actualRoomPos.setTableAngle_deg(270);
    m_planSetupPos.x = 0.00001;
    m_planSetupPos.y = 0.00001;
    m_planSetupPos.z = 0.00001;
    m_planSetupPos.rot = 0.00001;
    m_planSetupPos.p = 0.00001;
    m_planSetupPos.r = 0.00001;
    m_planIsoPos.x = 0.00001;
    m_planIsoPos.y = 0.00001;
    m_planIsoPos.z = 0.00001;
    m_planIsoPos.rot = 270.0;
    m_planIsoPos.p = 0.00001;
    m_planIsoPos.r = 0.00001;
    m_isValidPlanData=false;
    m_preppedForMotion = false;
}

/**
 * @brief Deselect Gantry Commands.
 * @details Set all Gantry/Extension Checkboxes to false.
 */
void DdsSimulator::deselectGantryCmds()
{
    gantrySelect->setChecked(false);
    gantry90Select->setChecked(false);
    gantry180Select->setChecked(false);
    gExtendSelect->setChecked(false);
}

/**
 * @brief Deselect all Couch Commands
 * @details Set all the Couch Checkboxes to false.
 */
void DdsSimulator::deselectCouchCmds()
{
    groupBox_4->setChecked(false);
    couchLoadSelect->setChecked(false);
    couchSaveSelect->setChecked(false);
    couchRecallSelect->setChecked(false);
    couchSwitchSelect->setChecked(false);
}

/**
 * @brief Update all the Couch/Cig values and publish Couch/Cig actuals on DDS.
 * @note Called on all changes as well as periodically by the ticTimer.
 */
void DdsSimulator::update()
{
    srsdds::CouchCig data;
    
    data.info.state = (srsdds::couchState)this->couchInfo->currentIndex();
    data.preppedForMotion = m_preppedForMotion;
    data.isPendantEnable = m_preppedForMotion;
    data.isTTCalibratedFresh = true;
    data.programMoveComplete =
            m_couchIOStatus.appStatusBits & COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;

    static CouchMode storedMode = eSetup;

    if ( m_mode == eTx ) {
        if ( storedMode != eTx )
        {
            // We just reset the isocenter, so clear out any iso offsets in the plan position.
            m_planIsoPos.x = 0;
            m_planIsoPos.y = 0;
            m_planIsoPos.z = 0;

            // Update the isocenter we're using.
            updateAndPublishIsocenter();
        }

        data.type = srsdds::ePAT;
    }
    else
    {
        data.type = srsdds::eROOM;
    }

    data.room.x = m_actualRoomPos.x_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.room.y = m_actualRoomPos.y_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.room.z = m_actualRoomPos.z_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.room.pitch = m_actualRoomPos.pitch_deg();
    data.room.roll = m_actualRoomPos.roll_deg();
    data.room.turntable = m_actualRoomPos.tableAngle_deg();

    // Calculate the isocentric coords, TT coords and calibrated coords.
    mev::TableTopGeometry ttGeom(m_actualRoomPos);
    data.tableTop.lateral = ttGeom.lateral_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.tableTop.longitudinal = ttGeom.longitudinal_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.tableTop.vertical = ttGeom.vertical_mm() * CM_TO_MM_CONVERSION_FACTOR;

    mev::TableTopGeometry isoGeom(m_actualRoomPos);
    isoGeom.setLateral_mm(isoGeom.lateral_mm() - m_ttIsocenter.lateral_mm());
    isoGeom.setLongitudinal_mm(isoGeom.longitudinal_mm() - m_ttIsocenter.longitudinal_mm());
    isoGeom.setVertical_mm(isoGeom.vertical_mm() - m_ttIsocenter.vertical_mm());

    data.iso.lateral = isoGeom.lateral_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.iso.longitudinal = isoGeom.longitudinal_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.iso.vertical = isoGeom.vertical_mm() * CM_TO_MM_CONVERSION_FACTOR;
    data.iso.turntable = isoGeom.tableAngle_deg();
    data.iso.pitch = isoGeom.pitch_deg();
    data.iso.roll = isoGeom.roll_deg();

    data.calibratedTabletop.lateral = data.tableTop.lateral;
    data.calibratedTabletop.longitudinal = data.tableTop.longitudinal;
    data.calibratedTabletop.vertical = data.tableTop.vertical;
    data.calibratedTabletop.turntable = data.room.turntable;
    data.calibratedTabletop.pitch = data.room.pitch;
    data.calibratedTabletop.roll = data.room.roll;

    data.gantryAngle = m_actualGantryPos;
    data.gantryExtension = m_actualGantryExt * CM_TO_MM_CONVERSION_FACTOR;
    
    locationTableWidget->setItem(0,0,new QTableWidgetItem(QString::number(data.room.x)));
    locationTableWidget->setItem(0,1,new QTableWidgetItem(QString::number(data.room.y)));
    locationTableWidget->setItem(0,2,new QTableWidgetItem(QString::number(data.room.z)));
    locationTableWidget->setItem(0,3,new QTableWidgetItem(QString::number(data.room.pitch)));
    locationTableWidget->setItem(0,4,new QTableWidgetItem(QString::number(data.room.roll)));
    locationTableWidget->setItem(0,5,new QTableWidgetItem(QString::number(data.room.turntable)));

    locationTableWidget->setItem(1,0,new QTableWidgetItem(QString::number(data.iso.lateral)));
    locationTableWidget->setItem(1,1,new QTableWidgetItem(QString::number(data.iso.longitudinal)));
    locationTableWidget->setItem(1,2,new QTableWidgetItem(QString::number(data.iso.vertical)));
    locationTableWidget->setItem(1,3,new QTableWidgetItem(QString::number(data.iso.pitch)));
    locationTableWidget->setItem(1,4,new QTableWidgetItem(QString::number(data.iso.roll)));
    locationTableWidget->setItem(1,5,new QTableWidgetItem(QString::number(data.iso.turntable)));

    locationTableWidget->setItem(2,0,new QTableWidgetItem(QString::number(data.tableTop.lateral)));
    locationTableWidget->setItem(2,1,new QTableWidgetItem(QString::number(data.tableTop.longitudinal)));
    locationTableWidget->setItem(2,2,new QTableWidgetItem(QString::number(data.tableTop.vertical)));

    locationTableWidget->setItem(3,0,new QTableWidgetItem(QString::number(m_ttIsocenter.lateral_mm() * CM_TO_MM_CONVERSION_FACTOR)));
    locationTableWidget->setItem(3,1,new QTableWidgetItem(QString::number(m_ttIsocenter.longitudinal_mm() * CM_TO_MM_CONVERSION_FACTOR)));
    locationTableWidget->setItem(3,2,new QTableWidgetItem(QString::number(m_ttIsocenter.vertical_mm() * CM_TO_MM_CONVERSION_FACTOR)));

    m_couchCigPub->write( data, DDS_HANDLE_NIL );

    storedMode = m_mode;
}

/**
 * @brief Set the Couch Load Position and publish it on DDS.
 * @details Set by the 'real' HandPendant if the Serial Port is initalized or by the GUI buttons.
 */
void DdsSimulator::setCouchLoadType() {
    qDebug() << "setCouchLoadType()";

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        couchLoadSelect->setChecked(true);
        cmd = eLoad;
        point_6d loadPos6D = RoomGeoToPoint6D(load_pos);
        emitCouchSetpoint(srsdds::eCOUCH_LOAD, &loadPos6D);
    }
    else
    {
        if (couchLoadSelect->checkState() == Qt::Checked) {
            cmd = eLoad;
            point_6d loadPos6D = RoomGeoToPoint6D(load_pos);
            emitCouchSetpoint(srsdds::eCOUCH_LOAD, &loadPos6D);
        }
        else if (couchSetupSelect->isChecked()) {
            cmd = eGotoSetup;
            emitCouchSetpoint(srsdds::eCOUCH_SETUP, &m_planSetupPos);
        }
        else if (couchTxSelect->isChecked()) {
            cmd = eGotoTx;
            emitCouchSetpoint(srsdds::eCOUCH_TREATMENT, &m_planIsoPos);
        }
    }

    deselectGantryCmds();
    couchSaveSelect->setChecked(false);
    couchRecallSelect->setChecked(false);
    couchSwitchSelect->setChecked(false);
}

/**
 * @brief Set and publish on DDS the Couch Save position.
 * @note Mirror real Hand Pendant buttons if attached.
 */
void DdsSimulator::setCouchSaveType() {
    qDebug() << "setCouchSaveType()";

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        couchSaveSelect->setChecked(true);
    }

    if (couchSaveSelect->checkState() == Qt::Checked)
    {
        m_savePos = m_actualRoomPos;
        couchSaveSelect->setChecked(false);
    }
    else if (couchSetupSelect->isChecked())
    {
        cmd = eGotoSetup;
    }
    else if (couchTxSelect->isChecked())
    {
        cmd = eGotoTx;
    }

    deselectGantryCmds();
    couchLoadSelect->setChecked(false);
    couchRecallSelect->setChecked(false);
    couchSwitchSelect->setChecked(false);
}

/**
 * @brief Set and publish on DDS the Couch Recall state.
 * @note Mirror real Hand Pendant buttons if attached.
 */
void DdsSimulator::setCouchRecallType() {
    qDebug() << "setCouchRecallType()";

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        couchRecallSelect->setChecked(true);
        cmd = eRecall;
    }
    else
    {
        if (couchRecallSelect->checkState() == Qt::Checked) {
            cmd = eRecall;
        }
        else if (couchSetupSelect->isChecked()) {
            cmd = eGotoSetup;
        }
        else if (couchTxSelect->isChecked()) {
            cmd = eGotoTx;
        }
    }
    
    deselectGantryCmds();
    couchLoadSelect->setChecked(false);
    couchSaveSelect->setChecked(false);
    couchSwitchSelect->setChecked(false);
}

/**
 * @brief Set and publish on DDS the Couch Switch state.
 * @note Mirror real Hand Pendant buttons if attached.
 */

void DdsSimulator::setCouchSwitchType() {
    qDebug() << "setCouchSwitchType()";

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        couchSwitchSelect->setChecked(true);
        cmd = eSwitch;
    }
    else
    {
        if (couchSwitchSelect->checkState() == Qt::Checked) {
            cmd = eSwitch;
        }
        else if (couchSetupSelect->isChecked()) {
            cmd = eGotoSetup;
        }
        else if (couchTxSelect->isChecked()) {
            cmd = eGotoTx;
        }
    }
    
    deselectGantryCmds();
    couchLoadSelect->setChecked(false);
    couchSaveSelect->setChecked(false);
    couchRecallSelect->setChecked(false);
}

/**
 * @brief Set and publish on DDS the Gantry Angle Type.
 * @note Mirror real Hand Pendant buttons if attached.
 */

void DdsSimulator::setGantryAngleType()
{

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        gantrySelect->setChecked(true);
        cmd = eAngle;
    }
    else
    {
        if (gantrySelect->checkState() == Qt::Checked)
        {
            cmd = eAngle;
        }
        else if (couchSetupSelect->isChecked())
        {
            cmd = eGotoSetup;
        }
        else if (couchTxSelect->isChecked())
        {
            cmd = eGotoTx;
        }
    }

    gantry90Select->setChecked(false); // disable if no destination
    gantry180Select->setChecked(false); // disable if no destination
    gExtendSelect->setChecked(false); // disable if no destination
    if (!m_isValidPlanData) {
        gantrySelect->setChecked(false); // disable if no destination
    }
    deselectCouchCmds();
    qDebug() << "Angle pressed";
}

/**
 * @brief Set Gantry Angle to 90 degrees.
 * @note Mirror real Hand Pendant buttons if attached.
 */
 void DdsSimulator::setGantry90Type()
{

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        gantry90Select->setChecked(true);
        cmd = e90;
    }
    else
    {
        if (gantry90Select->checkState() == Qt::Checked)
        {
            cmd = e90;
        }
        else if (couchSetupSelect->isChecked())
        {
            cmd = eGotoSetup;
        }
        else if (couchTxSelect->isChecked())
        {
            cmd = eGotoTx;
        }
    }

    deselectCouchCmds();
    gantry180Select->setChecked(false); // disable if no destination
    gExtendSelect->setChecked(false); // disable if no destination
    gantrySelect->setChecked(false); // disable if no destination
    qDebug() << "Angle 90 pressed";
}

/**
 * @brief Set Gantry Angle to 180 degrees.
 * @note Mirror real Hand Pendant buttons if attached.
 */
void DdsSimulator::setGantry180Type()
{

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        gantry180Select->setChecked(true);
        cmd = e180;
    }
    else
    {
        if (gantry180Select->checkState() == Qt::Checked)
        {
            cmd = e180;
        }
        else if (couchSetupSelect->isChecked())
        {
            cmd = eGotoSetup;
        }
        else if (couchTxSelect->isChecked())
        {
            cmd = eGotoTx;
        }
    }

    deselectCouchCmds();
    gantry90Select->setChecked(false); // disable if no destination
    gExtendSelect->setChecked(false); // disable if no destination
    gantrySelect->setChecked(false); // disable if no destination
    qDebug() << "Angle 180 pressed";
}

/**
 * @brief Set Gantry Gantry Extension Type.
 * @note Mirror real Hand Pendant buttons if attached.
 */

void DdsSimulator::setGantryExtendType()
{

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        gExtendSelect->setChecked(true);
        cmd = eExtend;
    }
    else
    {
        if (gExtendSelect->checkState() == Qt::Checked) {
            cmd = eExtend;
        }
        else if (couchSetupSelect->isChecked()) {
            cmd = eGotoSetup;
        }
        else if (couchTxSelect->isChecked()) {
            cmd = eGotoTx;
        }
    }

    deselectCouchCmds();
    gantry90Select->setChecked(false); // disable if no destination
    gantry180Select->setChecked(false); // disable if no destination
    gantrySelect->setChecked(false); // disable if no destination
    qDebug() << "Extension pressed";
}

/**
 * @brief Set Gantry Setup Type.
 * @note Mirror real Hand Pendant buttons if attached.
 */
void DdsSimulator::setCouchSetupType() {
    qDebug() << "setCouchSetupType()";

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        couchSetupSelect->setChecked(true);
    }

    m_mode = eSetup;
    cmd = eGotoSetup;
    deselectCouchCmds();
    deselectGantryCmds();
    setSetupLabels();
    update();
    
    emitCouchSetpoint1(srsdds::eGANTRY_EXTEND, m_planGantryExt);
    emitCouchSetpoint1(srsdds::eGANTRY_ANGLE, m_planGantryPos);
    emitCouchSetpoint(srsdds::eCOUCH_SETUP, &m_planSetupPos);
}

/**
 * @brief Set Gantry Treatment Type.
 * @note Mirror real Hand Pendant buttons if attached.
 */
void DdsSimulator::setCouchTxType()
{
    qDebug() << "setCouchTxType()";

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        couchTxSelect->setChecked(true);
    }

    m_mode = eTx;
    cmd = eGotoTx;
    deselectCouchCmds();
    setTxLabels();
    deselectGantryCmds();
    update();
    
    emitCouchSetpoint1(srsdds::eGANTRY_EXTEND, m_planGantryExt);
    emitCouchSetpoint1(srsdds::eGANTRY_ANGLE, m_planGantryPos);
    emitCouchSetpoint(srsdds::eCOUCH_SETUP, &m_planIsoPos);
}

/**
 * @brief Set Gantry Delta Type.
 * @note Mirror real Hand Pendant buttons if attached.
 */
void DdsSimulator::setCouchDeltaType()
{
    qDebug() << "setCouchDeltaType()";

    if (m_serialPortInitialized)
    {
        // Mirror real pendant button press on GUI
        couchDeltaSelect->setChecked(true);
    }

    deselectCouchCmds();
    deselectGantryCmds();
    if (m_isDeltaValid) {
        //    mode = eDelta;
        cmd = eGotoDelta;
        return;
    }
    if (m_mode == eTx)
    {
        couchTxSelect->setChecked(true);
    } else if (m_mode == eSetup) {
        couchSetupSelect->setChecked(true);
    }
}

/**
 * @brief Set the labels for SETUP mode.
 * @details Primarily, this means use X,Y,Z for Room Coordinates in SETUP mode.
 */
void DdsSimulator::setSetupLabels()
{
    latLabel->setText(QApplication::translate("MainWindow", "X", 0));
    longLabel->setText(QApplication::translate("MainWindow", "Y", 0));
    vertLabel->setText(QApplication::translate("MainWindow", "Z", 0));
    //groupBox_5->setStyleSheet("background-color: #729fcf;");
    jogPlusPitch->setEnabled(true);
    jogMinusPitch->setEnabled(true);
    jogPlusRoll->setEnabled(true);
    jogMinusRoll->setEnabled(true);
    label_3->setEnabled(true);
    label_4->setEnabled(true);
}

/**
 * @brief Set the labels for TREATMENT mode.
 * @details Primarily, this means use Lateral,Longitudinal and Vertical for Isocenter Coordinates
 * coordinates in TREATMENT mode.
 */
void DdsSimulator::setTxLabels() {
    latLabel->setText(QApplication::translate("MainWindow", "Lat", 0));
    longLabel->setText(QApplication::translate("MainWindow", "Long", 0));
    vertLabel->setText(QApplication::translate("MainWindow", "Vert", 0));
    //groupBox_5->setStyleSheet("background-color: #8ae234;");
    jogPlusPitch->setDisabled(true);
    jogMinusPitch->setDisabled(true);
    jogPlusRoll->setDisabled(true);
    jogMinusRoll->setDisabled(true);
    label_3->setDisabled(true);
    label_4->setDisabled(true);
}

/**
 * @brief Callback when the tic Timer expires.
 * @details If a button is currently active (depressed), perform the appropriate action
 * such as advancing a couch position, moving to a pre-programmed position, etc.
 * Call update() and the end.
 * @sa HandPendant::update().
 */
void DdsSimulator::timerUpdate()
{
    m_timetic++;
    
    if (m_activeButton == eNone)
    {
        if(m_preppedForMotion == true)
        {
            m_preppedForMotion = false;
            update();
        }
        return;
    }
    
    switch (m_activeButton)
    {
        case ePlusGA:
            moveGantryAngle(0.6);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualGantryPos, 'f', 2));
            break;
            
        case eMinusGA:
            moveGantryAngle(-0.6);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualGantryPos, 'f', 2));
            break;
            
        case ePlusGExt:
            moveGantryExtension(0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_EXTENSION_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualGantryExt, 'f', 2));
            break;
            
        case eMinusGExt:
            moveGantryExtension(-0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_EXTENSION_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualGantryExt, 'f', 2));
            break;
            
        case ePlusRot:
            moveRotation(0.6);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.tableAngle_deg(), 'f', 2));
            break;
            
        case eMinusRot:
            moveRotation(-0.6);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.tableAngle_deg(), 'f', 2));
            break;
            
        case ePlusZ:
            moveZ((QApplication::keyboardModifiers() & Qt::ShiftModifier) ? -1.0 : -0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.z_mm(), 'f', 2));
            break;
            
        case eMinusZ:
            moveZ((QApplication::keyboardModifiers() & Qt::ShiftModifier) ? -1.0 : -0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.z_mm(), 'f', 2));
            break;
            
        case ePlusX:            
            moveXorLateral((QApplication::keyboardModifiers() & Qt::ShiftModifier) ? 1.0 : 0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.x_mm(), 'f', 2));
            break;
            
        case eMinusX:
            moveXorLateral((QApplication::keyboardModifiers() & Qt::ShiftModifier) ? -1.0 : -0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.x_mm(), 'f', 2));
            break;
            
        case ePlusY:
            moveYorLongitudinal((QApplication::keyboardModifiers() & Qt::ShiftModifier) ? 1.0 : 0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.y_mm(), 'f', 2));
            break;
            
        case eMinusY:
            moveYorLongitudinal((QApplication::keyboardModifiers() & Qt::ShiftModifier) ? -1.0 : -0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.y_mm(), 'f', 2));
            break;
            
        case ePlusPitch:
            movePitch(0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.pitch_deg(), 'f', 2));
            break;
            
        case eMinusPitch:
            movePitch(-0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;

            testTimer->setText(QString::number(m_actualRoomPos.pitch_deg(), 'f', 2));
            break;
            
        case ePlusRoll:
            moveRoll(0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;

            testTimer->setText(QString::number(m_actualRoomPos.roll_deg(), 'f', 2));
            break;
            
        case eMinusRoll:
            moveRoll(-0.1);
            m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
            
            testTimer->setText(QString::number(m_actualRoomPos.roll_deg(), 'f', 2));
            break;
            
        case eMove:
            switch (cmd)
            {
                case e90:
                    // first retract
                    if (m_actualGantryExt < maxExt)
                    {
                        retractApplicator();
                        return;
                    }
                    if (m_actualGantryPos < 90)
                    {
                        m_actualGantryPos += 0.6;
                    } else {
                        m_actualGantryPos -= 0.6;
                    }
                    if (mev::approximatelyEqual(m_actualGantryPos, 90, 0.61))
                    {
                        m_actualGantryPos = 90;
                        m_couchIOStatus.appStatusBits |= COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
                    }
                    else
                    {
                        m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
                    }
                    
                    testTimer->setText(QString::number(m_actualGantryPos, 'f', 2));
                    break;
                    
                case e180:
                    // first retract
                    if (m_actualGantryExt < maxExt)
                    {
                        retractApplicator();
                        return;
                    }
                    if (m_actualGantryPos < 180)
                    {
                        m_actualGantryPos += 0.6;
                    }
                    else
                    {
                        m_actualGantryPos -= 0.6;
                    }
                    if (mev::approximatelyEqual(m_actualGantryPos, 180, 0.61))
                    {
                        m_actualGantryPos = 180;
                        m_couchIOStatus.appStatusBits |= COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
                    }
                    else
                    {
                        m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
                    }
                    
                    
                    testTimer->setText(QString::number(m_actualGantryPos, 'f', 2));
                    break;
                    
                case eLoad:
                    {
                        point_6d loadPos6D = RoomGeoToPoint6D(load_pos);
                        advanceCouchSetupMode(loadPos6D);
                        break;
                    }
                    
                case eGotoSetup:
                    if (m_isSetupValid)
                    {
                        advanceCouchSetupMode(m_planSetupPos);
                    }
                    break;
                    
                case eGotoTx:
                    if (m_isTxValid)
                    {
                        advanceCouchTxMode(m_planIsoPos);
                    }
                    break;
                    
                case eGotoDelta:
                    if (m_isDeltaValid)
                    {
                        advanceCouchDeltaMode();
                    }
                    break;
                    
                case eAngle:
                    // first retract
                    if (m_actualGantryExt < maxExt)
                    {
                        retractApplicator();
                        return;
                    }
                    if (m_actualGantryPos < m_planGantryPos)
                    {
                        m_actualGantryPos += 1.1;
                    }
                    else
                    {
                        m_actualGantryPos -= 1.1;
                    }
                    if (mev::approximatelyEqual(m_actualGantryPos, m_planGantryPos, 1.2))
                    {
                        m_actualGantryPos = m_planGantryPos;
                        m_couchIOStatus.appStatusBits |= COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
                    }
                    else
                    {
                        m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
                    }
                    break;
                    
                case eExtend:
                    if (m_actualGantryExt < m_planGantryExt)
                    {
                        m_actualGantryExt += 1.1;
                    }
                    else
                    {
                        m_actualGantryExt -= 1.1;
                    }
                    if (mev::approximatelyEqual(m_actualGantryExt, m_planGantryExt, 1.11))
                    {
                        m_actualGantryExt = m_planGantryExt;
                        m_couchIOStatus.appStatusBits |= COUCHIO_CIG_EXTENSION_MOVE_COMPLETE;
                    }
                    else
                    {
                        m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_EXTENSION_MOVE_COMPLETE;
                    }
                    break;
                    
                case eRecall:
                    {
                        point_6d savePos6D = RoomGeoToPoint6D(m_savePos);
                        advanceCouchSetupMode(savePos6D);
                        if (m_actualRoomPos.approximatelyEqual(m_savePos, 0.01) )
                        {
                            qDebug() << "Calibrated Move Complete";
                            m_couchIOStatus.appStatusBits |= COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
                            qDebug() << "case eRecall set COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE "
                            << (m_couchIOStatus.appStatusBits &
                                COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE);
                            if (couchSetupSelect->isChecked())
                                cmd = eGotoSetup;
                            else if (couchTxSelect->isChecked())
                                cmd = eGotoTx;
                            couchRecallSelect->setChecked(false);
                        }
                        else {
                            m_couchIOStatus.appStatusBits ^=
                                ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
                        }
                    }
                    break;
                default:
                    qDebug() << "Unknown cmd";
            }
            break;
            
        case eBeamPrep:
            if (m_timetic == 20)
            {
                // activate beam prep here
                m_handPendantCmd.handPendantCommand = srsdds::e_DONE;
                m_handPendantCmdsPub->write( m_handPendantCmd, DDS_HANDLE_NIL );
                
                testTimer->setText(QApplication::translate("MainWindow", "BeamPrep", 0));
            }
            break;
            
        default:
            QString t = testTimer->text();
            if (t.contains("123"))
            {
                testTimer->setText(QApplication::translate("MainWindow", "TextLabel", 0));
            }
            else
            {
                testTimer->setText(QApplication::translate("MainWindow", "1234567890", 0));
            }
    }
    
    // Publish the AppStatus
    {
        if(pmcSelect->isChecked())
        {
            mev::setBits(m_couchIOStatus.appStatusBits, COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE);
        }
        else
        {
            mev::removeBits(m_couchIOStatus.appStatusBits, COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE);
        }

        if(atScanPosCheckBox->isChecked())
        {
            mev::setBits(m_couchIOStatus.appStatusBits, COUCHIO_AT_CT_SCAN_POSITION);
        }
        else
        {
            mev::removeBits(m_couchIOStatus.appStatusBits, COUCHIO_AT_CT_SCAN_POSITION);
        }

        qDebug() << "Publishing app status";
        m_appStatusPub->write( m_couchIOStatus, DDS_HANDLE_NIL );
    }
    
    update();
    return;
    
    QString t = testTimer->text();
    if (t.contains("123"))
    {
        testTimer->setText(QApplication::translate("MainWindow", "TextLabel", 0));
    }
    else
    {
        testTimer->setText(QApplication::translate("MainWindow", "1234567890", 0));
    }
}

/**
 * @brief Retract the Applicator.
 * @details I.e., move towards the Maximum Snout Position.
 */
void DdsSimulator::retractApplicator() {
    QString s;
    m_actualGantryExt += .1;
    if (m_actualGantryExt > maxExt) {
        m_actualGantryExt = maxExt;
        m_couchIOStatus.appStatusBits |= COUCHIO_CIG_EXTENSION_MOVE_COMPLETE;
    }
    else {
        m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_EXTENSION_MOVE_COMPLETE;
    }
    
    testTimer->setText(s.number(m_actualGantryExt, 'f', 2));
    update();
}

/**
 * @brief Activate the Move Button.
 */
void DdsSimulator::movePressed() {
    qDebug() << "movePressed()";
    m_activeButton = eMove;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Move Button.
 */
void DdsSimulator::moveReleased() {
    qDebug() << "moveReleased()";
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Gantry Angle (+) Jog Button
 */
void DdsSimulator::jogPlusGAPressed() {
    qDebug() << "jogPlusGAPressed()";
    m_activeButton = ePlusGA;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Gantry Angle (+) Jog Button.
 */
void DdsSimulator::jogPlusGAReleased()
{
    qDebug() << "jogPlusGAReleased()";

    if (m_timetic == 0)
    {
        moveGantryAngle(0.1);
        testTimer->setText(QString::number(m_actualGantryPos, 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Gantry Angle (-) Jog Button
 */
void DdsSimulator::jogMinusGAPressed()
{
    qDebug() << "jogMinusGAPressed()";
    m_activeButton = eMinusGA;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Gantry Angle (-) Jog Button.
 */
void DdsSimulator::jogMinusGAReleased()
{
    qDebug() << "jogMinusGAReleased()";
    if (m_timetic == 0)
    {
        moveGantryAngle(-0.1);
        testTimer->setText(QString::number(m_actualGantryPos, 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Gantry Extension (+) Jog Button
 */
void DdsSimulator::jogPlusGExtPressed()
{
    qDebug() << "jogPlusGExtPressed()";
    m_activeButton = ePlusGExt;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Gantry Extension (+) Jog Button
 */
void DdsSimulator::jogPlusGExtReleased()
{
    qDebug() << "jogPlusGExtReleased()";
    if (m_timetic == 0)
    {
        moveGantryExtension(0.05);
        testTimer->setText(QString::number(m_actualGantryExt, 'f', 2));
        update();
    }
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Gantry Extension (-) Jog Button
 */
void DdsSimulator::jogMinusGExtPressed()
{
    qDebug() << "jogMinusGExtPressed()";
    m_activeButton = eMinusGExt;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Gantry Extension (-) Jog Button
 */
void DdsSimulator::jogMinusGExtReleased()
{
    qDebug() << "jogMinusGExtReleased()";
    if (m_timetic == 0)
    {
        moveGantryExtension(-0.05);
        testTimer->setText(QString::number(m_actualGantryExt, 'f', 2));
        update();
    }
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch X (+) Jog Button
 */
void DdsSimulator::jogPlusXPressed()
{
    qDebug() << "jogPlusXPressed()";
    m_activeButton = ePlusX;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch X (+) Jog Button
 */
void DdsSimulator::jogPlusXReleased()
{
    qDebug() << "jogPlusXReleased()";
    if (m_timetic == 0)
    {
        moveXorLateral(0.05);
        testTimer->setText(QString::number(m_actualRoomPos.x_mm(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch X (-) Jog Button
 */
void DdsSimulator::jogMinusXPressed()
{
    qDebug() << "jogMinusXPressed()";
    m_activeButton = eMinusX;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch X (+) Jog Button
 */
void DdsSimulator::jogMinusXReleased()
{
    qDebug() << "jogMinusXReleased()";
    if (m_timetic == 0)
    {
        moveXorLateral(-0.05);
        testTimer->setText(QString::number(m_actualRoomPos.x_mm(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch X Zero Button.
 */
void DdsSimulator::zeroXPressed()
{
    qDebug() << "zeroXPressed()";
    const double curX = m_actualRoomPos.x_mm();
    moveXorLateral(-curX);
    update();
}

/**
 * @brief Activate the Couch Y (+) Jog Button
 */
void DdsSimulator::jogPlusYPressed()
{
    qDebug() << "jogPlusYPressed()";
    m_activeButton = ePlusY;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Y (+) Jog Button
 */
void DdsSimulator::jogPlusYReleased()
{
    qDebug() << "jogPlusYReleased()";
    if (m_timetic == 0)
    {
        moveYorLongitudinal(0.05);
        testTimer->setText(QString::number(m_actualRoomPos.y_mm(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Y (-) Jog Button
 */
void DdsSimulator::jogMinusYPressed()
{
    qDebug() << "jogMinusYPressed()";
    m_activeButton = eMinusY;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Y (-) Jog Button
 */
void DdsSimulator::jogMinusYReleased()
{
    qDebug() << "jogMinusYReleased()";
    if (m_timetic == 0)
    {
        moveYorLongitudinal(-0.05);
        testTimer->setText(QString::number(m_actualRoomPos.y_mm(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Y Zero Button.
 */
void DdsSimulator::zeroYPressed()
{
    qDebug() << "zeroYPressed()";
    const double curY = m_actualRoomPos.y_mm();
    moveYorLongitudinal(-curY);
    update();
}

/**
 * @brief Activate the Couch Z (+) Jog Button
 */
void DdsSimulator::jogPlusZPressed()
{
    qDebug() << "jogPlusZPressed()";
    m_activeButton = ePlusZ;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Z (+) Jog Button
 */
void DdsSimulator::jogPlusZReleased()
{
    qDebug() << "jogPlusZReleased()";
    if (m_timetic == 0)
    {
        moveZ(0.05);
        testTimer->setText(QString::number(m_actualRoomPos.z_mm(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Z (-) Jog Button
 */
void DdsSimulator::jogMinusZPressed() {
    qDebug() << "jogMinusZPressed()";
    m_activeButton = eMinusZ;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Z (-) Jog Button
 */
void DdsSimulator::jogMinusZReleased()
{
    qDebug() << "jogMinusZReleased()";
    if (m_timetic == 0)
    {
        moveZ(-0.05);
        testTimer->setText(QString::number(m_actualRoomPos.z_mm(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Z Zero Button.
 */
void DdsSimulator::zeroZPressed()
{
    qDebug() << "zeroZPressed()";
    const double curZ = m_actualRoomPos.z_mm();
    moveZ(-curZ);
    update();
}

/**
 * @brief Activate the Couch Rotation (+) Jog Button
 */
void DdsSimulator::jogPlusRotPressed()
{
    qDebug() << "jogPlusRotPressed()";
    m_activeButton = ePlusRot;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Rotation (+) Jog Button
 */
void DdsSimulator::jogPlusRotReleased()
{
    qDebug() << "jogPlusRotReleased()";
    if (m_timetic == 0) {
        moveRotation(0.3);
        testTimer->setText(QString::number(m_actualRoomPos.tableAngle_deg(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Rotation (-) Jog Button
 */
void DdsSimulator::jogMinusRotPressed()
{
    qDebug() << "jogMinusRotPressed()";
    m_activeButton = eMinusRot;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Rotation (-) Jog Button
 */
void DdsSimulator::jogMinusRotReleased()
{
    qDebug() << "jogMinusRotReleased()";
    if (m_timetic == 0)
    {
        moveRotation(-0.3);
        testTimer->setText(QString::number(m_actualRoomPos.tableAngle_deg(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Pitch (+) Jog Button
 */
void DdsSimulator::jogPlusPitchPressed()
{
    qDebug() << "jogPlusPitchPressed()";
    m_activeButton = ePlusPitch;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Pitch (+) Jog Button
 */
void DdsSimulator::jogPlusPitchReleased()
{
    qDebug() << "jogPlusPitchReleased()";
    if (m_timetic == 0)
    {
        movePitch(0.05);
        testTimer->setText(QString::number(m_actualRoomPos.pitch_deg(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Pitch (-) Jog Button
 */
void DdsSimulator::jogMinusPitchPressed()
{
    qDebug() << "jogMinusPitchPressed()";
    m_activeButton = eMinusPitch;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Pitch (-) Jog Button
 */
void DdsSimulator::jogMinusPitchReleased()
{
    qDebug() << "jogMinusPitchReleased()";
    if (m_timetic == 0)
    {
        movePitch(-0.05);
        testTimer->setText(QString::number(m_actualRoomPos.pitch_deg(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Roll (+) Jog Button
 */
void DdsSimulator::jogPlusRollPressed()
{
    qDebug() << "jogPlusRollPressed()";
    m_activeButton = ePlusRoll;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Roll (+) Jog Button
 */
void DdsSimulator::jogPlusRollReleased()
{
    qDebug() << "jogPlusRollReleased()";
    if (m_timetic == 0) {
        moveRoll(0.05);
        testTimer->setText(QString::number(m_actualRoomPos.roll_deg(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Activate the Couch Roll (-) Jog Button
 */
void DdsSimulator::jogMinusRollPressed() {
    qDebug() << "jogMinusRollPressed()";
    m_activeButton = eMinusRoll;
    m_timer->start(100);
}

/**
 * @brief Deactivate the Couch Roll (-) Jog Button
 */
void DdsSimulator::jogMinusRollReleased() {
    qDebug() << "jogMinusRollReleased()";
    if (m_timetic == 0) {
        moveRoll(-0.05);
        testTimer->setText(QString::number(m_actualRoomPos.roll_deg(), 'f', 2));
        update();
    }
    
    m_activeButton = eNone;
    m_timer->stop();
    m_timetic=0;
}

/**
 * @brief Exit the application.
 * @details Will be called if the user closes the window containing the HandPendant.
 */
void DdsSimulator::closeEvent( QCloseEvent *event )
{
    (void)event;
    
    exit(0);
}

/**
 * @brief Compares two point_6d for near equality
 * @param src First point
 * @param dest Second point
 * @return True if the points are within 0.01 units for all 6 coords.
 */
bool DdsSimulator::isPointEqual(point_6d src, point_6d dest)
{
    return
        mev::approximatelyEqual(src.x, dest.x, 0.01) &&
        mev::approximatelyEqual(src.y, dest.y, 0.01) &&
        mev::approximatelyEqual(src.z, dest.z, 0.01) &&
        mev::approximatelyEqual(src.rot, dest.rot, 0.01) &&
        mev::approximatelyEqual(src.p, dest.p, 0.01) &&
        mev::approximatelyEqual(src.r, dest.r, 0.01);
    
}

/**
 * @brief Move the Couch towards the programmed Setup Position.
 */
void DdsSimulator::advanceCouchSetupMode(point_6d dest)
{
    bool setupComplete = true;
    
    QString s;
    double dx =  dest.x - m_actualRoomPos.x_mm();
    double dy =  dest.y - m_actualRoomPos.y_mm();
    double dz =  dest.z - m_actualRoomPos.z_mm();
    double stepsize = 0.5;
    double stepsize_c = 0.51;
    double stepsize_rot = 1.0;
    double stepsize_rot_c = 1.1;

    MEV_LOG_DEBUG("dx: " << dx);
    MEV_LOG_DEBUG("dy: " << dy);
    MEV_LOG_DEBUG("dz: " << dz);
    if (dx < 0)
    {
        m_actualRoomPos.setX_mm(m_actualRoomPos.x_mm() - stepsize);
    }
    else
    {
        m_actualRoomPos.setX_mm(m_actualRoomPos.x_mm() + stepsize);
    }
    if (mev::approximatelyEqual(m_actualRoomPos.x_mm(), dest.x, stepsize_c))
    {
        m_actualRoomPos.setX_mm(dest.x);
    }
    else
    {
        setupComplete = false;
    }
    
    m_actualRoomPos.setX_mm(mev::math::clamp<double>(m_actualRoomPos.x_mm(), minX, maxX));
    
    testTimer->setText(s.number(m_actualRoomPos.x_mm(), 'f', 2));
    
    if (dy < 0)
    {
        m_actualRoomPos.setY_mm(m_actualRoomPos.y_mm() - stepsize );
    }
    else
    {
        m_actualRoomPos.setY_mm(m_actualRoomPos.y_mm() + stepsize );
    }
    if (mev::approximatelyEqual(m_actualRoomPos.y_mm(), dest.y, stepsize_c))
    {
        m_actualRoomPos.setY_mm(dest.y);
    }
    else
    {
        setupComplete = false;
    }
    
    m_actualRoomPos.setY_mm( mev::math::clamp<double>(m_actualRoomPos.y_mm(), minY, maxY) );
    
    if (dz < 0)
    {
        m_actualRoomPos.setZ_mm(m_actualRoomPos.z_mm() - stepsize );
    }
    else
    {
        m_actualRoomPos.setZ_mm(m_actualRoomPos.z_mm() + stepsize );
    }
    if (mev::approximatelyEqual(m_actualRoomPos.z_mm(), dest.z, stepsize_c))
    {
        m_actualRoomPos.setZ_mm( dest.z );
    }
    else
    {
        setupComplete = false;
    }
    
    m_actualRoomPos.setZ_mm( mev::math::clamp<double>(m_actualRoomPos.z_mm(), minZ, maxZ) );
    
    double rot = mev::approximatelyEqual(dest.rot, 0, stepsize_c) ?
                 360.0 :
                 mev::math::roundAngle0to360(dest.rot);
    double drtn =  rot - m_actualRoomPos.tableAngle_deg();
    
    if (drtn < 0.0)
    {
        m_actualRoomPos.setTableAngle_deg(m_actualRoomPos.tableAngle_deg() - stepsize_rot );
    }
    else
    {
        m_actualRoomPos.setTableAngle_deg(m_actualRoomPos.tableAngle_deg() + stepsize_rot );
    }

    if (m_actualRoomPos.tableAngle_deg() > 360.0)
    {
        m_actualRoomPos.setTableAngle_deg(0.0);
    }
    
    if (mev::approximatelyEqual(m_actualRoomPos.tableAngle_deg(), dest.rot, stepsize_rot_c))
    {
        m_actualRoomPos.setTableAngle_deg( dest.rot );
    }
    else
    {
        setupComplete = false;
    }
    m_actualRoomPos.setTableAngle_deg( mev::math::roundAngle0to360(m_actualRoomPos.tableAngle_deg()));
    
    double dp = dest.p - m_actualRoomPos.pitch_deg();
    
    if (dp < 0.0)
    {
        m_actualRoomPos.setPitch_deg(m_actualRoomPos.pitch_deg() - stepsize );
    }
    else
    {
        m_actualRoomPos.setPitch_deg(m_actualRoomPos.pitch_deg() + stepsize );
    }
    
    if (mev::approximatelyEqual(m_actualRoomPos.pitch_deg(), dest.p, stepsize_c))
    {
        m_actualRoomPos.setPitch_deg( dest.p );
    }
    else
    {
        setupComplete = false;
    }
    
    m_actualRoomPos.setPitch_deg( mev::math::clamp<double>(m_actualRoomPos.pitch_deg(), minPitch, maxPitch) );
    
    double dr = dest.r - m_actualRoomPos.roll_deg();
    if (dr < 0)
    {
        m_actualRoomPos.setRoll_deg(m_actualRoomPos.roll_deg() - stepsize);
    }
    else
    {
        m_actualRoomPos.setRoll_deg(m_actualRoomPos.roll_deg() + stepsize);
    }
    
    if (mev::approximatelyEqual(m_actualRoomPos.roll_deg(), dest.r, stepsize_c))
    {
        m_actualRoomPos.setRoll_deg(dest.r);
    }
    else
    {
        setupComplete = false;
    }
    
    m_actualRoomPos.setRoll_deg(mev::math::clamp<double>(m_actualRoomPos.roll_deg(), minRoll, maxRoll));
    
    if ( setupComplete )
    {
        qDebug() << "Calibrated Move Complete.\n";
        m_couchIOStatus.appStatusBits |= COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
         qDebug() << "advanceCouchSetupMode set COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE " <<
            (m_couchIOStatus.appStatusBits & COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE);
        
    }
    else
    {
        m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    }
    
    //  update();
}

/**
 * @brief Move the Couch towards the programmed Treatment position.
 */
void DdsSimulator::advanceCouchTxMode(point_6d dest)
{
    QString s;
    
    bool txMoveComplete = true;

    // Generate an isocentric version of the table top coordinates and use them for all processing
    // in Tx mode.
    mev::TableTopGeometry actualIsoPos(m_actualRoomPos);
    actualIsoPos.setLateral_mm(actualIsoPos.lateral_mm() - m_ttIsocenter.lateral_mm());
    actualIsoPos.setLongitudinal_mm(actualIsoPos.longitudinal_mm() - m_ttIsocenter.longitudinal_mm());
    actualIsoPos.setVertical_mm(actualIsoPos.vertical_mm() - m_ttIsocenter.vertical_mm());

    // Treat the destination x, y and z as lat, long, vert.
    double dx =  dest.x - actualIsoPos.lateral_mm();
    double dy =  dest.y - actualIsoPos.longitudinal_mm();
    double dz =  dest.z - actualIsoPos.vertical_mm();
    double stepsize = 0.5;
    double stepsize_c = 0.51;
    double stepsize_rot = 2.5;
    double stepsize_rot_c = 2.6;

    qDebug() << Q_FUNC_INFO << dx << dy << dz;
    
    if (dx < 0.0)
    {
        actualIsoPos.setLateral_mm(actualIsoPos.lateral_mm() - stepsize);
    }
    else
    {
        actualIsoPos.setLateral_mm(actualIsoPos.lateral_mm() + stepsize);
    }
    if (mev::approximatelyEqual(actualIsoPos.lateral_mm(), dest.x, stepsize_c))
    {
        actualIsoPos.setLateral_mm(dest.x);
    }
    else
    {
        txMoveComplete = false;
    }
    
    actualIsoPos.setLateral_mm(mev::math::clamp<double>(actualIsoPos.lateral_mm(), minX, maxX) );
    
    testTimer->setText(s.number(actualIsoPos.lateral_mm(), 'f', 2));
    
    if (dy < 0.0)
    {
        actualIsoPos.setLongitudinal_mm(actualIsoPos.longitudinal_mm() - stepsize );
    }
    else
    {
        actualIsoPos.setLongitudinal_mm(actualIsoPos.longitudinal_mm() + stepsize );
    }

    if (mev::approximatelyEqual(actualIsoPos.longitudinal_mm(), dest.y, stepsize_c))
    {
        actualIsoPos.setLongitudinal_mm(dest.y);
    }
    else
    {
        txMoveComplete = false;
    }
    
    actualIsoPos.setLongitudinal_mm(mev::math::clamp<double>(actualIsoPos.longitudinal_mm(), minY, maxY));
    
    if (dz < 0.0)
    {
        actualIsoPos.setVertical_mm(actualIsoPos.vertical_mm() - stepsize );
    }
    else
    {
        actualIsoPos.setVertical_mm(actualIsoPos.vertical_mm() + stepsize );
    }
    
    if (mev::approximatelyEqual(actualIsoPos.vertical_mm(), dest.z, stepsize_c))
    {
        actualIsoPos.setVertical_mm(dest.z);
    }
    else
    {
        txMoveComplete = false;
    }
    
    actualIsoPos.setVertical_mm(mev::math::clamp<double>(actualIsoPos.vertical_mm(), minZ, maxZ));
    
    double rot = mev::approximatelyEqual(dest.rot, 0.0, stepsize_c) ? 360.0 : dest.rot;
    double drtn =  rot - actualIsoPos.tableAngle_deg();

    if (fabs(drtn) > 180.0)
    {
        drtn = -drtn;
    }
    
    if (drtn < 0.0)
    {
        actualIsoPos.setTableAngle_deg(actualIsoPos.tableAngle_deg() - stepsize_rot );
    }
    else
    {
        actualIsoPos.setTableAngle_deg(actualIsoPos.tableAngle_deg() + stepsize_rot );
    }
    
    if (actualIsoPos.tableAngle_deg() > 360.0)
    {
        actualIsoPos.setTableAngle_deg(0.0);
    }
    
    if (mev::approximatelyEqual(actualIsoPos.tableAngle_deg(), dest.rot, stepsize_rot_c))
    {
        actualIsoPos.setTableAngle_deg(dest.rot);
    }
    else
    {
        txMoveComplete = false;
    }
    
    actualIsoPos.setTableAngle_deg(mev::math::roundAngle0to360(actualIsoPos.tableAngle_deg()) );
    
    double dp = dest.p - actualIsoPos.pitch_deg();
    if (dp < 0)
    {
        actualIsoPos.setPitch_deg(actualIsoPos.pitch_deg() - stepsize );
    }
    else
    {
        actualIsoPos.setPitch_deg(actualIsoPos.pitch_deg() + stepsize );
    }
    
    if (mev::approximatelyEqual(actualIsoPos.pitch_deg(), dest.p, stepsize_c))
    {
        actualIsoPos.setPitch_deg(dest.p);
    }
    else
    {
        txMoveComplete = false;
    }
    
    actualIsoPos.setPitch_deg( mev::math::clamp<double>(actualIsoPos.pitch_deg(), minPitch, maxPitch) );
    
    double dr = dest.r - actualIsoPos.roll_deg();
    if (dr < 0)
    {
        actualIsoPos.setRoll_deg(actualIsoPos.roll_deg() - stepsize);
    }
    else
    {
        actualIsoPos.setRoll_deg(actualIsoPos.roll_deg() + stepsize);
    }
    
    if (mev::approximatelyEqual(actualIsoPos.roll_deg(), dest.r, stepsize_c))
    {
        actualIsoPos.setRoll_deg(dest.r);
    }
    else
    {
        txMoveComplete = false;
    }
    
    actualIsoPos.setRoll_deg(mev::math::clamp<double>(actualIsoPos.roll_deg(), minRoll, maxRoll));
    
    if ( txMoveComplete )
    {
        qDebug() << Q_FUNC_INFO << "Calibrated Move Complete.\n";
        m_couchIOStatus.appStatusBits |= COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    }
    else
    {
        m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    }

    // Convert back to room geometry via TT geometry.
    actualIsoPos.setLateral_mm(actualIsoPos.lateral_mm() + m_ttIsocenter.lateral_mm());
    actualIsoPos.setLongitudinal_mm(actualIsoPos.longitudinal_mm() + m_ttIsocenter.longitudinal_mm());
    actualIsoPos.setVertical_mm(actualIsoPos.vertical_mm() + m_ttIsocenter.vertical_mm());

    m_actualRoomPos = mev::RoomGeometry(actualIsoPos);
    
    update();
    
}

/**
 * @brief Initialize the Setup Field
 */
void DdsSimulator::init_setupData(void)
{
    m_planSetupPos.x = 10.0;
    m_planSetupPos.y = -10.0;
    m_planSetupPos.z = 15.0;
    m_isSetupValid = true;
    cmd = eGotoSetup;
}

/**
 * @brief DDS callback SLOT for when BeamInfo topic has been received.
 * @details Receipt of this topic means a new field has been loaded. Determine
 * whether it is a SETUP or TREATMENT field and configure couch parameters
 * appropriately including updating the Hand Pendant Presets.
 */
void DdsSimulator::onPlanBeamReceived()
{
    m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_ANGLE_MOVE_COMPLETE;
    m_couchIOStatus.appStatusBits &= ~COUCHIO_CIG_EXTENSION_MOVE_COMPLETE;

    qDebug() << "setBeamInfo set COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE " <<
        (m_couchIOStatus.appStatusBits & COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE);

    // Get DDS data
    PlanBeam planBeam;
    m_planBeamListener->getData( planBeam );
    
    if ( planBeam.isPlanInfo )
    {
        // Do nothing and wait for the beam info.
        return;
    }
    m_couchIOStatus.appStatusBits = 0;
    
    QString tdtString(planBeam.beamInfo.TreatmentDeliveryType);
    couchDeltaSelect->setCheckable(false);
    m_isDeltaValid = false;
    srsdds::CouchCigSetpoint newPoint;
    memset(&newPoint, 0, sizeof(srsdds::CouchCigSetpoint));
    
    if (tdtString.contains("SETUP", Qt::CaseInsensitive))
    {
        m_isSetupValid = true;
        m_isTxValid = false;
        m_planSetupPos.x = planBeam.beamInfo.TableTopLateral_mm / CM_TO_MM_CONVERSION_FACTOR;
        m_planSetupPos.y = planBeam.beamInfo.TableTopLongitudinal_mm / CM_TO_MM_CONVERSION_FACTOR;
        m_planSetupPos.z = planBeam.beamInfo.TableTopVertical_mm / CM_TO_MM_CONVERSION_FACTOR;
        m_planSetupPos.rot = mev::math::roundAngle0to360( planBeam.beamInfo.TableTopSupportAngle_deg );
        m_planSetupPos.p = planBeam.beamInfo.TableTopPitchAngle_deg;
        m_planSetupPos.r = planBeam.beamInfo.TableTopRollAngle_deg;
        beamReceived->setText(QString("Setup"));
        newPoint.presetType = srsdds::eCOUCH_SETUP;
        newPoint.x = m_planSetupPos.x * CM_TO_MM_CONVERSION_FACTOR;
        newPoint.y = m_planSetupPos.y * CM_TO_MM_CONVERSION_FACTOR;
        newPoint.z = m_planSetupPos.z * CM_TO_MM_CONVERSION_FACTOR;
        newPoint.rtn = mev::math::roundAngle0to360( m_planSetupPos.rot );
        newPoint.pitch = m_planSetupPos.p;
        newPoint.roll = m_planSetupPos.r;
        m_planGantryExt = 33.6;
        m_mode = eSetup;
        cmd = eGotoSetup;

        srsdds::CouchIsocenter couchIso;
        couchIso.room.x = newPoint.x;
        couchIso.room.y = newPoint.y;
        couchIso.room.z = newPoint.z;
        couchIso.room.pitch = newPoint.pitch;
        couchIso.room.roll = newPoint.roll;
        couchIso.room.turntable = newPoint.rtn;

        mev::RoomGeometry rg(
                    couchIso.room.x,
                    couchIso.room.y,
                    couchIso.room.z,
                    couchIso.room.turntable,
                    couchIso.room.pitch,
                    couchIso.room.roll
                    );
        mev::TableTopGeometry ttg(rg);
        couchIso.tableTop.lateral = ttg.lateral_mm();
        couchIso.tableTop.longitudinal = ttg.longitudinal_mm();
        couchIso.tableTop.vertical = ttg.vertical_mm();

        m_couchIsocenterPub->write(couchIso, DDS_HANDLE_NIL);
    }
    else
    {
        // Must be a Treatment Field
        m_isTxValid = true;
        m_planIsoPos.x = planBeam.beamInfo.TableTopLateral_mm / CM_TO_MM_CONVERSION_FACTOR;
        m_planIsoPos.y = planBeam.beamInfo.TableTopLongitudinal_mm / CM_TO_MM_CONVERSION_FACTOR;
        m_planIsoPos.z = planBeam.beamInfo.TableTopVertical_mm / CM_TO_MM_CONVERSION_FACTOR;
        m_planIsoPos.rot = mev::math::roundAngle0to360( planBeam.beamInfo.TableTopSupportAngle_deg );
        m_planIsoPos.p = planBeam.beamInfo.TableTopPitchAngle_deg;
        m_planIsoPos.r = planBeam.beamInfo.TableTopRollAngle_deg;

        qDebug() << Q_FUNC_INFO << m_planIsoPos.x << m_planIsoPos.y << m_planIsoPos.z;

        beamReceived->setText(QString("Tx"));
        newPoint.presetType = srsdds::eCOUCH_TREATMENT;
        newPoint.x = m_planIsoPos.x * CM_TO_MM_CONVERSION_FACTOR;
        newPoint.y = m_planIsoPos.y * CM_TO_MM_CONVERSION_FACTOR;
        newPoint.z = m_planIsoPos.z * CM_TO_MM_CONVERSION_FACTOR;
        newPoint.rtn = mev::math::roundAngle0to360( m_planIsoPos.rot );
        newPoint.pitch = m_planIsoPos.p;
        newPoint.roll = m_planIsoPos.r;
        m_planGantryExt = planBeam.beamInfo.SnoutPosition / CM_TO_MM_CONVERSION_FACTOR;
        m_mode = eTx;
        cmd = eGotoTx;
    }
    memcpy(&m_couchPresetLocationInstance, &newPoint, sizeof(srsdds::CouchCigSetpoint));

    m_couchCigSetpointPub->write( m_couchPresetLocationInstance, DDS_HANDLE_NIL );
    
    m_planGantryPos = planBeam.beamInfo.GantryAngle_deg;
    emitCouchSetpoint1(srsdds::eGANTRY_ANGLE, m_planGantryPos);
    
    qDebug() << "Snout Position: " << m_planGantryExt;
    emitCouchSetpoint1(srsdds::eGANTRY_EXTEND, m_planGantryExt);
 
    m_isValidPlanData = true;
    gantrySelect->setCheckable(true);
    gExtendSelect->setCheckable(true);
    
    m_couchIOStatus.appStatusBits |= COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    
    m_appStatusPub->write( m_couchIOStatus, DDS_HANDLE_NIL );
}

/**
 * @brief DDS callback SLOT called when Couch Cig Setpoint data has been received.
 * @details Set the HandPendant Presets for Couch Cig moves with the new values.
 * @note This is being used for testing the CTCalibrator, which sends everything as a setup move
 * right now.
 */
void DdsSimulator::onCouchCigSetpointReceived()
{
    // Get DDS data
    srsdds::CouchCigSetpoint couchCigSetpoint;
    m_couchCigSetpointListener->getData( couchCigSetpoint );

    qDebug() << Q_FUNC_INFO << couchCigSetpoint.presetType;

    if ( couchCigSetpoint.presetType == srsdds::eCOUCH_SETUP)
    {
        cmd = eGotoSetup;
        m_planSetupPos.x = couchCigSetpoint.x / CM_TO_MM_CONVERSION_FACTOR;
        m_planSetupPos.y = couchCigSetpoint.y / CM_TO_MM_CONVERSION_FACTOR;
        m_planSetupPos.z = couchCigSetpoint.z / CM_TO_MM_CONVERSION_FACTOR;
        m_planSetupPos.p = couchCigSetpoint.pitch;
        m_planSetupPos.r = couchCigSetpoint.roll;
        m_planSetupPos.rot = couchCigSetpoint.rtn;
        m_isSetupValid = true;
    }
    else
    {
        MEV_LOG_INFO("CouchCigSetpoints handling for presets besides setup not yet implemented.");
    }
}

/**
 * @brief DDS callback SLOT called when Couch Deltas have been received from Verity.
 * @details Set the HandPendant Presets for Couch Delta mode with the new values received from
 * Verity.
 */
void DdsSimulator::onCouchDeltasReceived()
{
    // Get DDS data
    srsdds::CouchDelta couchDelta;
    m_couchDeltaListener->getData( couchDelta );
    
    qDebug() << Q_FUNC_INFO << couchDelta.dx << couchDelta.dy << couchDelta.dz;

    m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;

    if ( couchDelta.presetType == srsdds::eCOUCH_DELTA
        || couchDelta.presetType == srsdds::eCOUCH_DELTA_REGAPP )
    {
        m_isDeltaValid = true;
        m_planDeltaPos.x = m_actualRoomPos.x_mm() + couchDelta.dx/CM_TO_MM_CONVERSION_FACTOR;
        m_planDeltaPos.y = m_actualRoomPos.y_mm() + couchDelta.dy/CM_TO_MM_CONVERSION_FACTOR;
        m_planDeltaPos.z = m_actualRoomPos.z_mm() + couchDelta.dz/CM_TO_MM_CONVERSION_FACTOR;
        m_planDeltaPos.rot = m_actualRoomPos.tableAngle_deg() + couchDelta.drz;
        m_planDeltaPos.p = m_actualRoomPos.pitch_deg() + couchDelta.dry; // DRG fix 4/28/09: swap p and r to match CouchIO
        m_planDeltaPos.r = m_actualRoomPos.roll_deg() + couchDelta.drx;
        
        m_currentDeltaSet = mev::RoomGeometry(
                    couchDelta.dx/CM_TO_MM_CONVERSION_FACTOR,
                    couchDelta.dy/CM_TO_MM_CONVERSION_FACTOR,
                    couchDelta.dz/CM_TO_MM_CONVERSION_FACTOR,
                    couchDelta.drz,
                    couchDelta.dry,
                    couchDelta.drx
                    );

        couchDeltaSelect->setCheckable(true);
        deltaReceived->setText(QString("Delta"));
        
        srsdds::CouchCigSetpoint newPoint;
        memset(&newPoint, 0, sizeof(srsdds::CouchCigSetpoint));
        newPoint.presetType = srsdds::eCOUCH_DELTA_REGAPP;
        newPoint.x = couchDelta.dx;
        newPoint.y = couchDelta.dy;
        newPoint.z = couchDelta.dz;
        newPoint.rtn = couchDelta.drz;
        newPoint.pitch = couchDelta.dry;
        newPoint.roll = couchDelta.drx;
        
        memcpy(&m_couchPresetLocationInstance, &newPoint, sizeof(srsdds::CouchCigSetpoint));
        
        m_couchCigSetpointPub->write( m_couchPresetLocationInstance, DDS_HANDLE_NIL );
        
        cmd = eGotoDelta;
    }
    
    m_couchIOStatus.appStatusBits |= COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    
    m_appStatusPub->write( m_couchIOStatus, DDS_HANDLE_NIL );
}

/**
 * @brief Updates the isocenter we're using to be the current location of the couch.
 * @return A copy of the new isocenter.
 */
srsdds::CouchIsocenter DdsSimulator::updateAndPublishIsocenter()
{
    m_ttIsocenter = mev::TableTopGeometry(m_actualRoomPos);

    srsdds::CouchIsocenter couchIso;
    couchIso.room.x = m_actualRoomPos.x_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.room.y = m_actualRoomPos.y_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.room.z = m_actualRoomPos.z_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.room.pitch = m_actualRoomPos.pitch_deg();
    couchIso.room.roll = m_actualRoomPos.roll_deg();
    couchIso.room.turntable = m_actualRoomPos.tableAngle_deg();

    couchIso.tableTop.lateral = m_ttIsocenter.lateral_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.tableTop.longitudinal =
            m_ttIsocenter.longitudinal_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.tableTop.vertical = m_ttIsocenter.vertical_mm() * CM_TO_MM_CONVERSION_FACTOR;

    qDebug() << Q_FUNC_INFO << "Publishing new isocenter";
    m_couchIsocenterPub->write(couchIso, DDS_HANDLE_NIL);

    return couchIso;
}

/**
 * @brief If the ResetSession DDS topic contains a beam reset, publish the CouchDeltaLocation
 */
void DdsSimulator::resetSession()
{
    // Note: Defaulting to BEAM_RESET is just to stop GCC from complaining about
    // using an uninitialized variable...
    srsdds::SessionReset sessionReset;
    sessionReset.state = srsdds::SessionState::e_BEAM_RESET;
    if ( !m_sessionResetListener->getData(sessionReset) )
    {
        MEV_LOG_ERROR(
                    mev::CATEGORY_PROGRAM_FLOW,
                    mev::SEVERITY_CRITICAL,
                    "Failed to get SessionReset data"
                    );
        return;
    }

    if ( sessionReset.state == srsdds::e_BEAM_RESET )
    {
        srsdds::CouchDeltaLocation deltaLoc;
        deltaLoc.programMoveComplete =
                m_couchIOStatus.appStatusBits & COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;

        if ( m_couchPresetLocationInstance.presetType == srsdds::eCOUCH_SETUP ||
             m_couchPresetLocationInstance.presetType == srsdds::eCOUCH_DELTA)
        {
            // Update the isocenter to match the current location.
            srsdds::CouchIsocenter couchIso = updateAndPublishIsocenter();

            deltaLoc.location.lateral = couchIso.tableTop.lateral;
            deltaLoc.location.longitudinal = couchIso.tableTop.longitudinal;
            deltaLoc.location.vertical = couchIso.tableTop.vertical;
            deltaLoc.location.pitch = m_actualRoomPos.pitch_deg();
            deltaLoc.location.roll = m_actualRoomPos.roll_deg();
            deltaLoc.location.turntable = m_actualRoomPos.tableAngle_deg();
        }
        else
        {
            // Generate some TT coords to send as the current location.
            mev::RoomGeometry rg(
                        m_actualRoomPos.x_mm(),
                        m_actualRoomPos.y_mm(),
                        m_actualRoomPos.z_mm(),
                        m_actualRoomPos.tableAngle_deg(),
                        m_actualRoomPos.pitch_deg(),
                        m_actualRoomPos.roll_deg()
                        );
            mev::TableTopGeometry ttg(rg);

            deltaLoc.location.lateral = ttg.lateral_mm() * CM_TO_MM_CONVERSION_FACTOR;
            deltaLoc.location.longitudinal = ttg.longitudinal_mm() * CM_TO_MM_CONVERSION_FACTOR;
            deltaLoc.location.vertical = ttg.vertical_mm() * CM_TO_MM_CONVERSION_FACTOR;
            deltaLoc.location.pitch = m_actualRoomPos.pitch_deg();
            deltaLoc.location.roll = m_actualRoomPos.roll_deg();
            deltaLoc.location.turntable = m_actualRoomPos.tableAngle_deg();
        }

        // Ensure the correct isocenter is used. Note that we can't use the Room/TableTopGeometry
        // copy methods because of the conversion to mm here.
        m_ttIsocenter.getAsCouchTTStruct(deltaLoc.isocenter.tableTop);
        deltaLoc.isocenter.tableTop.lateral =
                m_ttIsocenter.lateral_mm() * CM_TO_MM_CONVERSION_FACTOR;
        deltaLoc.isocenter.tableTop.longitudinal =
                m_ttIsocenter.longitudinal_mm() * CM_TO_MM_CONVERSION_FACTOR;
        deltaLoc.isocenter.tableTop.vertical =
                m_ttIsocenter.vertical_mm() * CM_TO_MM_CONVERSION_FACTOR;

        mev::RoomGeometry roomIsocenter(m_ttIsocenter);
        deltaLoc.isocenter.room.x = roomIsocenter.x_mm() * CM_TO_MM_CONVERSION_FACTOR;
        deltaLoc.isocenter.room.y = roomIsocenter.y_mm() * CM_TO_MM_CONVERSION_FACTOR;
        deltaLoc.isocenter.room.z = roomIsocenter.z_mm() * CM_TO_MM_CONVERSION_FACTOR;
        deltaLoc.isocenter.room.turntable = roomIsocenter.tableAngle_deg();
        deltaLoc.isocenter.room.pitch = roomIsocenter.pitch_deg();
        deltaLoc.isocenter.room.roll = roomIsocenter.roll_deg();

        qDebug() << Q_FUNC_INFO << "Publishing CouchDeltaLocation"
                 << deltaLoc.location.lateral
                 << deltaLoc.location.longitudinal
                 << deltaLoc.location.vertical;


        // Give the isocenter time to update on TDS
        QThread::sleep(1);
        m_couchDeltaLocationPub->write(deltaLoc, DDS_HANDLE_NIL);
    }
}

/**
 * @brief Populate the CouchDeltaLocation DDS topic with the current couch location data, and
 *        publish it.
 */
void DdsSimulator::publishCapturedCouchLocation()
{
    srsdds::CouchDeltaLocation capturedCouchLocation;
    capturedCouchLocation.programMoveComplete =
            m_couchIOStatus.appStatusBits & COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;

    capturedCouchLocation.accurateRoomCoordsDirty = false; // assuming that couch does not move.

    srsdds::CouchIsocenter couchIso;
    couchIso.room.x = m_actualRoomPos.x_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.room.y = m_actualRoomPos.y_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.room.z = m_actualRoomPos.z_mm() * CM_TO_MM_CONVERSION_FACTOR;
    couchIso.room.pitch = m_actualRoomPos.pitch_deg();
    couchIso.room.roll = m_actualRoomPos.roll_deg();
    couchIso.room.turntable = m_actualRoomPos.tableAngle_deg();

    mev::RoomGeometry rg(
                couchIso.room.x,
                couchIso.room.y,
                couchIso.room.z,
                couchIso.room.turntable,
                couchIso.room.pitch,
                couchIso.room.roll
                );
    mev::TableTopGeometry ttg(rg);

    ttg.getAsCouchPatientStruct(capturedCouchLocation.location);

    DDS_ReturnCode_t retVal = m_CouchCapturedLocationPub->write(capturedCouchLocation,
                                                                DDS_HANDLE_NIL);
    if ( retVal != DDS_RETCODE_OK )
    {
        MEV_LOG_ERROR(mev::CATEGORY_IO_ERROR,
                      mev::SEVERITY_CRITICAL,
                      "Failed to send srsdds::CouchDeltaLocation.");
    }

    qDebug() <<  "Captured Couch Delta Location is sent." <<
                 " ProgramMoveComplete: " <<
                                (capturedCouchLocation.programMoveComplete ? "True" : "False") <<
                 " Location: X = " << capturedCouchLocation.location.lateral <<
                 " Location: Y = " << capturedCouchLocation.location.longitudinal <<
                 " Location: Z = " << capturedCouchLocation.location.vertical <<
                 " accurateRoomCoordsDirty: " <<
                                (capturedCouchLocation.accurateRoomCoordsDirty ? "True" : "False");
}

/**
 * @brief Slot called when the Xray Button is Clicked.
 * @details Simply reports via DDS that the button was selected.
 */
void DdsSimulator::xrayButtonClicked()
{
    qDebug() << "xrayButtonClicked() - unimplemented";
    m_handPendantCmd.handPendantCommand = srsdds::e_XRAY_IN;
    m_handPendantCmdsPub->write( m_handPendantCmd, DDS_HANDLE_NIL );
}

/**
 * @brief Move the couch towards the delta setpoint.
 */
void DdsSimulator::advanceCouchDeltaMode()
{
    QString s;
    point_6d roomPos = RoomGeoToPoint6D( m_actualRoomPos );
    point_6d deltaPos = RoomGeoToPoint6D( m_currentDeltaSet );
    bool changed=false;
    
    // To advance deltas, we move the current delta set in the opposite direction to the room coords
    if (!mev::approximatelyEqual(deltaPos.x, 0, 0.09))
    {
        if (deltaPos.x < 0)
        {
            roomPos.x -= 0.1;
            deltaPos.x += 0.1;
        }
        else
        {
            roomPos.x += 0.1;
            deltaPos.x -= 0.1;
        }
        roomPos.x = mev::math::clamp<double>(roomPos.x, minX, maxX);
        testTimer->setText(s.number(roomPos.x, 'f', 2));
        changed = true;
    }
    else
    {
        deltaPos.x = 0.0;
    }
    
    if (!mev::approximatelyEqual(deltaPos.y, 0, 0.09))
    {
        if (deltaPos.y < 0)
        {
            roomPos.y -= 0.1;
            deltaPos.y += 0.1;
        }
        else
        {
            roomPos.y += 0.1;
            deltaPos.y -= 0.1;
        }
        roomPos.y = mev::math::clamp<double>(roomPos.y, minY, maxY);
        testTimer->setText(s.number(roomPos.y, 'f', 2));
        changed = true;
    }
    else
    {
        deltaPos.y = 0;
    }
    
    if (!mev::approximatelyEqual(deltaPos.z, 0, 0.09))
    {
        if (deltaPos.z < 0)
        {
            roomPos.z -= 0.1;
            deltaPos.z += 0.1;
        } else
        {
            roomPos.z += 0.1;
            deltaPos.z -= 0.1;
        }
        roomPos.z = mev::math::clamp<double>(roomPos.z, minZ, maxZ);
        testTimer->setText(s.number(roomPos.z, 'f', 2));
        changed = true;
    }
    else
    {
        deltaPos.z = 0;
    }
    
    if (!mev::approximatelyEqual(deltaPos.rot, 0, 0.09))
    {
        if (deltaPos.rot < 0)
        {
            roomPos.rot -= 0.1;
            deltaPos.rot += 0.1;
            if (abs(roomPos.rot - maxRot) <= 0.1) {
                roomPos.rot = minRot;
            }
            roomPos.rot = mev::math::roundAngle0to360(roomPos.rot);
        }
        else
        {
            roomPos.rot += 0.1;
            deltaPos.rot -= 0.1;
            if (mev::approximatelyEqual(roomPos.rot, minRot, 0.1))
            {
                roomPos.rot = maxRot;
            }
            roomPos.rot = mev::math::roundAngle0to360(roomPos.rot);
        }
        testTimer->setText(s.number(roomPos.rot, 'f', 2));
        changed = true;
    }
    else
    {
        deltaPos.rot = 0;
    }
    
    if (!mev::approximatelyEqual(deltaPos.p, 0, 0.09))
    {
        if (deltaPos.p < 0)
        {
            roomPos.p -= 0.1;
            deltaPos.p += 0.1;
        }
        else
        {
            roomPos.p += 0.1;
            deltaPos.p -= 0.1;
        }
        roomPos.p = mev::math::clamp<double>(roomPos.p, minPitch, maxPitch);
        testTimer->setText(s.number(roomPos.p, 'f', 2));
        changed = true;
    }
    else
    {
        deltaPos.p = 0;
    }
    
    if (!mev::approximatelyEqual(deltaPos.r, 0, 0.09))
    {
        if (deltaPos.r < 0)
        {
            roomPos.r -= 0.1;
            deltaPos.r += 0.1;
        }
        else
        {
            roomPos.r += 0.1;
            deltaPos.r -= 0.1;
        }
        roomPos.r = mev::math::clamp<double>(roomPos.r, minRoll, maxRoll);
        testTimer->setText(s.number(roomPos.r, 'f', 2));
        changed = true;
    }
    else
    {
        deltaPos.r = 0;
    }
    
    m_actualRoomPos = mev::RoomGeometry(
                roomPos.x,
                roomPos.y,
                roomPos.z,
                roomPos.rot,
                roomPos.p,
                roomPos.r);
    m_currentDeltaSet = mev::RoomGeometry(
                deltaPos.x,
                deltaPos.y,
                deltaPos.z,
                deltaPos.rot,
                deltaPos.p,
                deltaPos.r);
    
    if (changed)
    {
        //  update();
        srsdds::CouchCigSetpoint newPoint;
        memset(&newPoint, 0, sizeof(srsdds::CouchCigSetpoint));
        newPoint.presetType = srsdds::eCOUCH_DELTA;
        newPoint.x = m_currentDeltaSet.x_mm();
        newPoint.y = m_currentDeltaSet.y_mm();
        newPoint.z = m_currentDeltaSet.z_mm();
        newPoint.rtn = m_currentDeltaSet.tableAngle_deg();
        newPoint.pitch = m_currentDeltaSet.pitch_deg();
        newPoint.roll = m_currentDeltaSet.roll_deg();
        
        memcpy(&m_couchPresetLocationInstance, &newPoint, sizeof(srsdds::CouchCigSetpoint));
        
        m_couchCigSetpointPub->write( m_couchPresetLocationInstance, DDS_HANDLE_NIL );
        
        m_couchIOStatus.appStatusBits &= ~COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    }
    else
    {
        qDebug() << "advanceCouchDeltaMode set COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE "
                 << (m_couchIOStatus.appStatusBits & COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE);
        qDebug() << "advanceCouchDeltaMode set COUCHIO_DELTA_MOVE_COMPLETE "
                 << (m_couchIOStatus.appStatusBits & COUCHIO_DELTA_MOVE_COMPLETE);
        m_couchIOStatus.appStatusBits |= COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
        m_couchIOStatus.appStatusBits |= COUCHIO_DELTA_MOVE_COMPLETE;
    }
}

/**
 * @brief Set the specified preset to the point and publish it on DDS.
 * @param preset The selected HandPendant Preset to update
 * @param p The point containing the preset coordinates
 */
void DdsSimulator::emitCouchSetpoint( srsdds::PresetID preset, const point_6d *p )
{
    memset( &m_couchPresetLocationInstance, 0, sizeof(srsdds::CouchCigSetpoint) );
    
    m_couchPresetLocationInstance.presetType = preset;
    m_couchPresetLocationInstance.x = p->x * CM_TO_MM_CONVERSION_FACTOR;
    m_couchPresetLocationInstance.y = p->y * CM_TO_MM_CONVERSION_FACTOR;
    m_couchPresetLocationInstance.z = p->z * CM_TO_MM_CONVERSION_FACTOR;
    m_couchPresetLocationInstance.rtn = mev::math::roundAngle0to360( p->rot );
    m_couchPresetLocationInstance.pitch = p->p;
    m_couchPresetLocationInstance.roll = p->r;
    
    m_couchCigSetpointPub->write( m_couchPresetLocationInstance, DDS_HANDLE_NIL );
    
}

/**
 * @brief Set the Specified Gantry Angle or Extension Presets.
 * @param preset eGANTRY_ANGLE or e_GANTRY_EXTEND
 * @param p Preset coordinates
 */
void DdsSimulator::emitCouchSetpoint1(srsdds::PresetID preset, const double p)
{
    
    if (preset == srsdds::eGANTRY_ANGLE)
    {
        memset(&m_gantryInstance, 0, sizeof(srsdds::CouchCigSetpoint));
        m_gantryInstance.presetType = preset;
        m_gantryInstance.theta = p;
        m_couchCigSetpointPub->write( m_gantryInstance, DDS_HANDLE_NIL );
    }
    else if (preset == srsdds::eGANTRY_EXTEND)
    {
        memset(&m_extensionInstance, 0, sizeof(srsdds::CouchCigSetpoint));
        m_extensionInstance.presetType = preset;
        m_extensionInstance.extension = p * CM_TO_MM_CONVERSION_FACTOR;
        m_couchCigSetpointPub->write( m_extensionInstance, DDS_HANDLE_NIL );
    }
    else
    {
        return;
    }
}

/**
 * @brief Forces an update based on the timer expiring.
 */
void DdsSimulator::ticUpdate()
{
    update();
}

/**
 * @brief Handle a Key Press Event
 * @details The following Keys are supported
 * - D Simulate Pressing BeamPrep
 * - X Exit the program
 */
void DdsSimulator::keyPressEvent(QKeyEvent *e)
{
    Qt::KeyboardModifiers modifiers = e->modifiers();

    switch (e->key())
    {
        case Qt::Key_D:
            qDebug() << "D ";
            beamPrepClicked();
            break;
            
        case Qt::Key_X:
            exit(0);

        case Qt::Key_T:
            if(modifiers & Qt::CTRL)
            {
                QRect geom = geometry();

                if ( locationTableWidget->isVisible() )
                {
                    geom.setHeight(1016);
                    locationTableWidget->setVisible(false);
                }
                else
                {
                    geom.setHeight(1200);
                    locationTableWidget->setVisible(true);
                }
                setGeometry(geom);
            }
        break;
    }
}

/**
 * @brief Callback when the Beam Prep button is clicked.
 * @details Publish on DDS the Hand Pendant Command that the Done/Beam Prep button has
 * been selected.
 */
void DdsSimulator::beamPrepClicked()
{
    // activate beam prep here
    m_handPendantCmd.handPendantCommand = srsdds::e_DONE;
    
    // This signifies that the Couch has actually reached desired position.  Call it true for now.
    m_handPendantCmd.condition = true;

    m_handPendantCmdsPub->write( m_handPendantCmd, DDS_HANDLE_NIL );
    
    qDebug() << "Emit BeamPrep";
    testTimer->setText(QApplication::translate("MainWindow", "BeamPrep", 0));

    if (m_serialPortInitialized)
    {
        // Also use this opportunity to send a stop command to the hub
        // (to stop attempting communication with the motor controller)
        sendSerialData();
    }
}

/**
 * @brief Forces an update based on the couch changing something we don't track.
 * @param state Unused.
 */
void DdsSimulator::couchStateChanged(int state) {
    (void)state;
    update();
}

/**
 * @brief Periodically send packet data to the hand pendant via serial
 */
void DdsSimulator::sendSerialData()
{
    // Send a command to the hub to stop attempting to connect to the motor controller via ethernet
    static std::string hccCmd = "hcc stop\r\n";

    qDebug() << "Sending stop command to hub...";
    if (m_serialPortPtr)
    {
        m_serialPortPtr->write(hccCmd.c_str(), hccCmd.length());
    }
}


/**
 * @brief Read the data available from the serial port and send DDS topics as required
 */
void DdsSimulator::readSerialData()
{
    // Keep track of the current position in the read buffer
    static int readPos = 0;

    bool processData = false;

    // Read all available incoming data from the serial port
    m_readDataBytes = m_serialPortPtr->readAll();

    if (m_readDataBytes.size() + readPos >= SERIAL_READ_BUFFER_SIZE_IN_BYTES)
    {
        qDebug() << "serial read buffer overflow condition! (read "
                 << m_readDataBytes.size() << " bytes from serial, read position="
                 << readPos << ")";

        // Clear any remaining data in the buffer
        ::memset(m_serialReadBuffer, 0, SERIAL_READ_BUFFER_SIZE_IN_BYTES);
        readPos = 0;

        return;
    }

    // The ethernet connection from the hub will periodically send out a single byte,
    // which we will want to avoid.
    //if (readDataBytes.size() == 1 && readDataBytes.at(0) == '.')
    //{
    //    qDebug() << "dumping '.'";
    //    return;
    //}

    // Transfer all bytes read to local storage
    for (int i = 0; i < m_readDataBytes.size(); ++i)
    {
        if (m_readDataBytes.at(i) != '\n')
        {
            m_serialReadBuffer[readPos] = m_readDataBytes.at(i);
            ++readPos;
        }
    }

    std::string pendantCmd(m_serialReadBuffer);

    // Look for specific strings in the serial data
    std::size_t clientNotRunningPos = pendantCmd.find("Client not running");
    std::size_t keyPos           = pendantCmd.rfind("hc_key6.5.4.3.2.1: ");
    std::size_t motionEnablePos  = pendantCmd.find("HC_MOTEN: 1");
    std::size_t motionDisablePos = pendantCmd.find("HC_MOTEN: 0");

    if (clientNotRunningPos != std::string::npos)
    {
        qDebug() << "chomping 'Client not running' from hub...";

        // Clear any remaining data in the buffer
        ::memset(m_serialReadBuffer, 0, SERIAL_READ_BUFFER_SIZE_IN_BYTES);
        readPos = 0;

        return;
    }

    // Look for the motion bar enable string first
    if (motionEnablePos != std::string::npos)
    {
        qDebug() << "Motion bars enable";
        on_prepForMotion_toggled(true);

        // Update the simulator GUI as well
        prepForMotion->setEnabled(false);
        update();

        // Clear any remaining data in the buffer
        ::memset(m_serialReadBuffer, 0, SERIAL_READ_BUFFER_SIZE_IN_BYTES);
        readPos = 0;

        return;
    }
    else if (motionDisablePos != std::string::npos)
    {
        qDebug() << "Motion bars disable";
        on_prepForMotion_toggled(false);

        // Update the simulator GUI as well
        prepForMotion->setEnabled(true);
        update();

        // Clear any remaining data in the buffer
        ::memset(m_serialReadBuffer, 0, SERIAL_READ_BUFFER_SIZE_IN_BYTES);
        readPos = 0;

        return;
    }

    if (keyPos != std::string::npos &&
        pendantCmd.substr(keyPos + LENGTH_OF_HEADER_STRING).size() >= 16)
    {
        qDebug() << "---------------------------";

        // Trim the heading string
        std::string subPos = pendantCmd.substr(keyPos + LENGTH_OF_HEADER_STRING);
        qDebug() << "Command: " << QString(subPos.c_str());

        try
        {
            // Grab each of the six bytes representing buttons -- "0.00.00.00.00.00"
            m_serialDataBytes[0] = strtol(subPos.substr(0,1).c_str(),  NULL, 16);
            m_serialDataBytes[1] = strtol(subPos.substr(2,2).c_str(),  NULL, 16);
            m_serialDataBytes[2] = strtol(subPos.substr(5,2).c_str(),  NULL, 16);
            m_serialDataBytes[3] = strtol(subPos.substr(8,2).c_str(),  NULL, 16);
            m_serialDataBytes[4] = strtol(subPos.substr(11,2).c_str(), NULL, 16);
            m_serialDataBytes[5] = strtol(subPos.substr(14,2).c_str(), NULL, 16);

            processData = true;
        }
        catch (std::out_of_range& oor)
        {
            qDebug() << "caught out_of_range exception, ignoring buffer contents...";
        }

        // Clear any remaining data in the buffer
        ::memset(m_serialReadBuffer, 0, SERIAL_READ_BUFFER_SIZE_IN_BYTES);
        readPos = 0;

        if (processData)
        {
            // Certain button presses should only be honored when the motion enable bars are enabled
            if (m_preppedForMotion)
            {
                processButton(5, 0x01, &DdsSimulator::jogPlusGAPressed,    &DdsSimulator::jogPlusGAReleased);      // Gantry Angle +
                processButton(5, 0x02, &DdsSimulator::jogPlusGExtPressed,  &DdsSimulator::jogPlusGExtReleased);    // Extension +
                processButton(5, 0x04, &DdsSimulator::jogPlusZPressed,     &DdsSimulator::jogPlusZReleased);       // Z / Vert +
                processButton(5, 0x08, &DdsSimulator::jogPlusYPressed,     &DdsSimulator::jogPlusYReleased);       // Y / Long +
                processButton(5, 0x10, &DdsSimulator::jogPlusXPressed,     &DdsSimulator::jogPlusXReleased);       // X / Lat +
                processButton(5, 0x20, &DdsSimulator::jogPlusRotPressed,   &DdsSimulator::jogPlusRotReleased);     // Rotate +
                processButton(5, 0x40, &DdsSimulator::jogPlusPitchPressed, &DdsSimulator::jogPlusPitchReleased);   // Pitch +

                processButton(4, 0x01, &DdsSimulator::jogMinusGAPressed,    &DdsSimulator::jogMinusGAReleased);    // Gantry Angle -
                processButton(4, 0x02, &DdsSimulator::jogMinusGExtPressed,  &DdsSimulator::jogMinusGExtReleased);  // Extension -
                processButton(4, 0x04, &DdsSimulator::jogMinusZPressed,     &DdsSimulator::jogMinusZReleased);     // Z / Vert -
                processButton(4, 0x08, &DdsSimulator::jogMinusYPressed,     &DdsSimulator::jogMinusYReleased);     // Y / Long -
                processButton(4, 0x10, &DdsSimulator::jogMinusXPressed,     &DdsSimulator::jogMinusXReleased);     // X / Lat -
                processButton(4, 0x20, &DdsSimulator::jogMinusRotPressed,   &DdsSimulator::jogMinusRotReleased);   // Rotate -
                processButton(4, 0x40, &DdsSimulator::jogMinusPitchPressed, &DdsSimulator::jogMinusPitchReleased); // Pitch -

                processButton(2, 0x10, &DdsSimulator::jogPlusRollPressed,  &DdsSimulator::jogPlusRollReleased);    // Roll Jog +
                processButton(2, 0x20, &DdsSimulator::jogMinusRollPressed, &DdsSimulator::jogMinusRollReleased);   // Roll Jog -
                processButton(2, 0x40, &DdsSimulator::movePressed,         &DdsSimulator::moveReleased);           // Move
            }

            // TODO:  The following won't compile
            
//            processButton(3, 0x01, &HandPendant::setCouchSetupType,  NO_CALLBACK_FUNCTION);                   // Set Up
//            processButton(3, 0x02, &HandPendant::setCouchTxType,     NO_CALLBACK_FUNCTION);                   // Treatment
//            processButton(3, 0x04, &HandPendant::setCouchDeltaType,  NO_CALLBACK_FUNCTION);                   // Delta
//            processButton(3, 0x08, &HandPendant::setCouchLoadType,   NO_CALLBACK_FUNCTION);                   // Load
//            processButton(3, 0x10, &HandPendant::setCouchSaveType,   NO_CALLBACK_FUNCTION);                   // Save
//            processButton(3, 0x20, &HandPendant::setCouchRecallType, NO_CALLBACK_FUNCTION);                   // Recall
//            processButton(3, 0x40, &HandPendant::setCouchSwitchType, NO_CALLBACK_FUNCTION);                   // Switch
//
//            processButton(2, 0x01, &HandPendant::setGantryAngleType,  NO_CALLBACK_FUNCTION);                  // Angle
//            processButton(2, 0x02, &HandPendant::setGantry90Type,     NO_CALLBACK_FUNCTION);                  // 90 deg
//            processButton(2, 0x04, &HandPendant::setGantry180Type,    NO_CALLBACK_FUNCTION);                  // 180 deg
//            processButton(2, 0x08, &HandPendant::setGantryExtendType, NO_CALLBACK_FUNCTION);                  // Extend
//
//            processButton(1, 0x01, NO_CALLBACK_FUNCTION,             NO_CALLBACK_FUNCTION);                    // Room Lights
//            processButton(1, 0x02, NO_CALLBACK_FUNCTION,             NO_CALLBACK_FUNCTION);                    // Laser
//            processButton(1, 0x04, NO_CALLBACK_FUNCTION,             NO_CALLBACK_FUNCTION);                    // Field Light
//            processButton(1, 0x08, &HandPendant::xrayButtonClicked, NO_CALLBACK_FUNCTION);                    // X-Ray In
//            processButton(1, 0x10, &HandPendant::beamPrepClicked,   NO_CALLBACK_FUNCTION);                    // Done / Beam Prep
//
//            processButton(0, 0x01, NO_CALLBACK_FUNCTION, NO_CALLBACK_FUNCTION);                                // Slow
//            processButton(0, 0x02, NO_CALLBACK_FUNCTION, NO_CALLBACK_FUNCTION);                                // Norm
//            processButton(0, 0x04, NO_CALLBACK_FUNCTION, NO_CALLBACK_FUNCTION);                                // Fast
        }
    }
}

/**
 * @brief Process a button press from the hand pendant
 * @param byteIndex The index into the serial data bytes
 * @param mask the bitmask to compare against
 * @param pressFP Function to be called when a button press is detected
 * @param releaseFP Function to be called when a button release is detected
 */
void DdsSimulator::processButton(int byteIndex,
                                 char mask,
                                 void (DdsSimulator::*pressFP)(),
                                 void (DdsSimulator::*releaseFP)())
{
    if (m_serialDataBytes[byteIndex] & mask)
    {
        // Toggle internal bit storage to represent "pressed"
        m_internalDataBytes[byteIndex] ^= mask;

        if (pressFP)
        {
            // Call the button press callback function
            (this->*pressFP)();
        }
    }
    else
    {
        if (m_internalDataBytes[byteIndex] & mask)
        {
            // Toggle internal bit storage to represent "released"
            m_internalDataBytes[byteIndex] ^= mask;

            if (releaseFP)
            {
                // Call the button release callback function
                (this->*releaseFP)();
            }
        }
    }
}

/**
 * @brief Initialize the serial port for reading hand pendant commands
 */
void DdsSimulator::initializeSerialPort()
{
    qDebug() << "Initializing serial port...";

    m_serialPortPtr = new QSerialPort(this);
    m_serialPortPtr->setPortName("/dev/ttyUSB0");
    m_serialPortPtr->setBaudRate(QSerialPort::Baud57600);
    m_serialPortPtr->setDataBits(QSerialPort::Data8);
    m_serialPortPtr->setParity(QSerialPort::NoParity);
    m_serialPortPtr->setStopBits(QSerialPort::OneStop);
    m_serialPortPtr->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serialPortPtr->open(QIODevice::ReadWrite))
    {
        qDebug() << "ERROR opening serial device: error="
                 << m_serialPortPtr->errorString();
    }
    else
    {
        qDebug() << "Successfully opened serial port for Read/Write";
        m_serialPortInitialized = true;
    }

    ::memset(m_internalDataBytes, 0, sizeof(m_internalDataBytes));
    ::memset(m_serialDataBytes,   0, sizeof(m_serialDataBytes));
}

/**
 * @brief Moves the gantry angle by step, clamping to its min/max values
 * @param step The angular distance in degrees by which to move the gantry
 */
void DdsSimulator::moveGantryAngle(double step)
{
    m_actualGantryPos += step;
    m_actualGantryPos = mev::math::clamp<double>(m_actualGantryPos, minGantry, maxGantry);
}

/**
 * @brief Moves the gantry extension by step, clamping to its min/max values
 * @param step The distance in mm by which to move the gantry
 */
void DdsSimulator::moveGantryExtension(double step)
{
    m_actualGantryExt += step;
    m_actualGantryExt = mev::math::clamp<double>(m_actualGantryExt, minExt, maxExt);
}

/**
 * @brief Moves X (room coordinates) if in setup mode, or Lateral (isocentric coordinates) if
 * in treatment mode, by step, clamping to its min/max values
 * @param step The distance in mm by which to move the couch along the X or Lateral axis
 */
void DdsSimulator::moveXorLateral(double step)
{
    if ( m_mode == eSetup || m_mode == eDelta )
    {
        // Room mode, so act directly on the room position x coordinate.
        double newX = mev::math::clamp<double>(m_actualRoomPos.x_mm() + step, minX, maxX);
        if (mev::approximatelyEqual(newX, 0, 0.001))
        {
            newX = 0.0;
        }
        m_actualRoomPos.setX_mm(newX);
    }
    else
    {
        // Treatment mode, so convert to TT and move laterally, then convert back.
        mev::TableTopGeometry ttGeom(m_actualRoomPos);

        double newLat = mev::math::clamp<double>(ttGeom.lateral_mm() + step, minX, maxX);
        if (mev::approximatelyEqual(newLat, 0, 0.001))
        {
            newLat = 0.0;
        }

        ttGeom.setLateral_mm(newLat);
        m_actualRoomPos = mev::RoomGeometry(ttGeom);
    }
}

/**
 * @brief Moves Y (room coordinates) if in setup mode, or Longitudinal (isocentric coordinates) if
 * in treatment mode, by step, clamping to its min/max values
 * @param step The distance in mm by which to move the couch along the Y or Longitudinal axis
 */
void DdsSimulator::moveYorLongitudinal(double step)
{
    if ( m_mode == eSetup || m_mode == eDelta )
    {
        // Room coordinates, so act directly on the room position y coordinate.
        double newY = mev::math::clamp<double>(m_actualRoomPos.y_mm() + step, minY, maxY);
        if (mev::approximatelyEqual(newY, 0, 0.001))
        {
            newY = 0.0;
        }
        m_actualRoomPos.setY_mm(newY);
    }
    else
    {
        // Convert to TT and move longitudinally, then convert back..
        mev::TableTopGeometry ttGeom(m_actualRoomPos);

        double newLong = mev::math::clamp<double>(ttGeom.longitudinal_mm() + step, minY, maxY);
        if (mev::approximatelyEqual(newLong, 0, 0.001))
        {
            newLong = 0.0;
        }

        ttGeom.setLongitudinal_mm(newLong);
        m_actualRoomPos = mev::RoomGeometry(ttGeom);
    }
}

/**
 * @brief Moves Z (room coordinates).
 * @param step The distance in mm by which to move the couch along the Z axis
 * @note The Z axis is always vertical, so this also moves the Vertical axis without needing any
 * rotations applied.
 */
void DdsSimulator::moveZ(double step)
{
    double newZ = mev::math::clamp<double>(m_actualRoomPos.z_mm() + step, minZ, maxZ);
    if (mev::approximatelyEqual(newZ, 0, 0.001))
    {
        newZ = 0.0;
    }
    m_actualRoomPos.setZ_mm(newZ);
}

/**
 * @brief Moves the couch rotation by step, ensuring it is not between 45 and 135 degrees
 * @param step The angular distance in degrees by which to move the gantry
 */
void DdsSimulator::moveRotation(double step)
{
    // Get a rotation angle that's within range .
    double rotAngle = mev::math::roundAngle0to360(m_actualRoomPos.tableAngle_deg() + step);

    if (mev::approximatelyEqual(rotAngle, minRot, (abs(step) + 0.01)))
    {
        rotAngle = minRot;
    }
    else if (mev::approximatelyEqual(m_actualRoomPos.tableAngle_deg(), maxRot, (abs(step) + 0.01)))
    {
        rotAngle = maxRot;
    }

    if (rotAngle > minRot && rotAngle < maxRot)
    {
        if (step < 0)
        {
            rotAngle = maxRot;
        }
        else
        {
            rotAngle = minRot;
        }
    }

    if ( m_mode == eSetup || m_mode == eDelta )
    {
        m_actualRoomPos.setTableAngle_deg(rotAngle);
    }
    else
    {
        // Treatment field, so we need to convert to tabletop coords before rotation.
        mev::TableTopGeometry ttGeom(m_actualRoomPos);
        ttGeom.setTableAngle_deg(rotAngle);
        m_actualRoomPos = mev::RoomGeometry(ttGeom);
    }
}

/**
 * @brief Moves the couch pitch by step
 * @param step The angular distance in degrees by which to move the gantry
 */
void DdsSimulator::movePitch(double step)
{
    m_actualRoomPos.setPitch_deg(
                mev::math::clamp<double>(m_actualRoomPos.pitch_deg() + step, minPitch, maxPitch)
                );
}

/**
 * @brief Moves the couch roll by step
 * @param step The angular distance in degrees by which to move the gantry
 */
void DdsSimulator::moveRoll(double step)
{
    m_actualRoomPos.setRoll_deg(
                mev::math::clamp<double>(m_actualRoomPos.roll_deg() + step, minRoll, maxRoll)
                );
}


/**
 * @brief Callback for when the Motion Enable Bars are pressed.
 * @details Does nothing for now.
 */
void DdsSimulator::motionEnablePressed()
{
    qDebug() << "motionEnablePressed()";
}

/**
 * @brief Callback for when the Emergency Stop button has been pressed.
 * @details Does nothing for now.
 */
void DdsSimulator::EStopPressed()
{
    qDebug() << "EStopPressed()";
}

void DdsSimulator::on_prepForMotion_toggled(bool enabled)
{
    m_preppedForMotion = enabled;
	QList<QWidget *> widgets = groupBox_5->findChildren<QWidget *>();
	QWidget *widget;
	foreach (widget, widgets)
    widget->setDisabled(!enabled);
	moveButton->setDisabled(!enabled);
	update();
}

/** @brief Handler for when PMC checkbox is clicked. */
void DdsSimulator::onPmcCheckboxClicked()
{
    // Just republish appStatus
    qDebug() << "Publishing app status";
    if(pmcSelect->isChecked())
    {
        m_couchIOStatus.appStatusBits |= COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE;
    }
    else
    {
        m_couchIOStatus.appStatusBits &= ~(COUCHIO_PROGRAMMED_CALIBRATED_MOVE_COMPLETE);
    }
    m_appStatusPub->write( m_couchIOStatus, DDS_HANDLE_NIL );
}

/** @brief Handler for when the At-Scan-Position button is clicked. */
void DdsSimulator::onSendScanPositionClicked()
{
    // Get data from combo
    QVariant comboData = scannerComboBox->currentData();
    if(comboData.isNull() || !comboData.isValid())
    {
        return;
    }

    // Get scanner ID
    const srsdds::ScannerId scannerId = comboData.value<srsdds::ScannerId>();

    // Get scan position
    mev::RoomGeometry ctScanPosition;
    m_scanPosConfig->scanPosition(scannerId, ctScanPosition);

    // Msg
    srsdds::CouchScannerResult msg;
    {
        msg.scannerId      = scannerId;
        msg.success        = true;
        msg.room.x         = ctScanPosition.x_mm();
        msg.room.y         = ctScanPosition.y_mm();
        msg.room.z         = ctScanPosition.z_mm();
        msg.room.turntable = ctScanPosition.tableAngle_deg();
        msg.room.pitch     = ctScanPosition.pitch_deg();
        msg.room.roll      = ctScanPosition.roll_deg();

        const mev::ScannerCalibration_v2* scannerCal = m_scanCalConfig->calibration(scannerId);
        MEV_ASSERT(scannerCal != nullptr);

        // Cal origin
        const mev::Vector3Dd& calOrigin_mm = scannerCal->calibrationOrigin_mm();
        mev::dds::vectorConvert(calOrigin_mm, msg.calOrigin_mm);

        // Cal Xform
        const mev::Matrix4x4& calXform = scannerCal->calibrationXform();
        mev::dds::matrixConvert(calXform, msg.calXform);
    }

    // Publish
    DDS_ReturnCode_t retVal = m_couchScannerResultPub->write(msg, DDS_HANDLE_NIL);
    if ( retVal != DDS_RETCODE_OK )
    {
        QString errMsg = QString("Failed to send srsdds::CouchScannerResult with error (%1).")
                                 .arg(retVal);
        MEV_LOG_ERROR(mev::CATEGORY_IO_ERROR, mev::SEVERITY_CRITICAL, errMsg.toStdString());
    }
}
