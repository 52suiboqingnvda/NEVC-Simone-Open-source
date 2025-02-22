
#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"
#include "SimOneEvaluationAPI.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "SampleGetNearMostLane.h"
#include "SampleGetLaneST.h"

#include <iostream>
#include <memory>


//HDMapStandalone::MSignal sign_flag;
//SSD::SimVector<HDMapStandalone::MSignal> sign_flag_list;
//Main function

int getcasenum() {
	SimOne_Data_CaseInfo pCaseInfoTest = SimOne_Data_CaseInfo();
	SimOneAPI::GetCaseInfo(&pCaseInfoTest);
	int num;
	sscanf(pCaseInfoTest.caseName,"%d",&num);
	return num;
}

int case_num = 0;

HDMapStandalone::MSignal GetTargetLight(const HDMapStandalone::MLaneId& id, const SSD::SimString& laneId, const SSD::SimVector<long>& roadIdList)//�źŵ�
{
	HDMapStandalone::MSignal light;
	SSD::SimVector<HDMapStandalone::MSignal> lightList;
	SimOneAPI::GetTrafficLightList(lightList);

	
	for (auto& item : lightList) {
		
	}
	for (auto& item : lightList)
	{
		int num = 0;
		
		for (auto& ptValidities : item.validities)
		{
			auto it = std::find(roadIdList.begin(), roadIdList.end(), ptValidities.roadId);
			if (it != roadIdList.end())
			{
				++num;
			}
		}

		//current lane
		if (num >= 2)
		{
			light = item;
			break;
		}
	}
	return std::move(light);
}

bool IsGreenLight(const long& lightId, const SSD::SimString& laneId, const HDMapStandalone::MSignal& light)//�ж��ǲ����̵�
{
	SimOne_Data_TrafficLight trafficLight;
	if (SimOneAPI::GetTrafficLight(0, lightId, &trafficLight))
	{
		if (trafficLight.status != ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green)
		{

			return false;
		}

		else
		{
			return int(trafficLight.countDown) > 5;
		}
	}
	else
	{
		return true;
	}
}

SSD::SimPoint3D GetTragetStopLine(const HDMapStandalone::MSignal& light, const SSD::SimString& laneId)//��ȡֹͣ��
{
	SSD::SimVector<HDMapStandalone::MObject> stoplineList;
	SimOneAPI::GetStoplineList(light, laneId, stoplineList);
	return stoplineList[0].pt;
}

void change_road_function(SSD::SimPoint3DVector * inps, SSD::SimPoint3D &start_p, SSD::SimPoint3D &end_p)
{
	inps->clear();
	inps->push_back(start_p);
	inps->push_back(end_p);
}

SSD::SimPoint3DVector inputPoints;
SSD::SimPoint3DVector targetPath;
SSD::SimPoint3D targetPointPos;


int main()
{
	bool inAEBState = false;//��ʾ�Ƿ���AEB״̬
	int timeout = 20;//��ʾ��ʱʱ��
	bool isSimOneInitialized = false;//ģ�����Ƿ��ѳ�ʼ��
	const char* MainVehicleId ="0";//��ʾ������ID
	bool isJoinTimeLoop = true; //��ʾ�Ƿ����ʱ��ѭ��
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop); //��ʼ��ģ����
	SimOneAPI::SetDriverName(MainVehicleId, "FirstAffection");//���ü�ʻԱ����Ϊ"FirstAffection"
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);//ʹ�ñ������ݳ�ʼ����������

	while (true) 
	{
		/*���ص�ͼ*/
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	SSD::SimPoint3DVector inputPoints;//����һ��3D�����������ڴ洢�����
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();//���ڴ洢·������Ϣ

	/*��ȡ��������·������Ϣ*/
	if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get()))
	{
		for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) {
			SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
			inputPoints.push_back(inputWayPoints);
			targetPointPos = inputPoints.back();
		}
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
		return -1;
	}

	/*����Ƿ�����Ч��·����*/
	SSD::SimPoint3DVector targetPath;
	if (pWayPoints->wayPointsSize >= 2) {
		SSD::SimVector<int> indexOfValidPoints;
		/*����·��*/
		if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
			return -1;
		}
	}

	/*/ �������ֻ��һ��·���㣬��ʹ�øõ�����ʼ������ID�ͳ�����ϢD*/
	else if (pWayPoints->wayPointsSize == 1) {
		SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
		HDMapStandalone::MLaneInfo laneInfoInit;
		if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle initial route failed");
			return -1;
		}
		else {
			targetPath = laneInfoInit.centerLine;
		}
	}

	/* ����Ƿ���·����*/
	else {			
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
		return -1;
	}
	

	case_num = getcasenum();

	int count = 0;

	while (true) {
		int frame = SimOneAPI::Wait();

		/*��鰸������״̬*/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop){
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		/*/��ȡ GPS ����*/
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId,pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		/*��ȡ�ϰ�������*/
		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId,pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		/*����Ƿ��������а���*/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);//ʹ�� GPS ���ݻ�ȡ����λ��
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);//���������ٶ�

			double minDistance = std::numeric_limits<double>::max();//��ʼ����С�����Ǳ���ϰ�������
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);//��ȡ�������ڵĳ��� ID
			SSD::SimString potentialObstacleLaneId = "";//��ʼ��Ǳ���ϰ��ﳵ�� ID
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);//��ȡ�ϰ����λ��
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);//��ȡ�ϰ������ڵĳ��� ID
				/*�ж��ϰ����Ƿ����������ڵĳ�����*/
				if (mainVehicleLaneId == obstacleLaneId) {
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);//�����ϰ���������֮���ƽ�����
					/*�ж��Ƿ�����ӽ����ϰ���*/
					if (obstacleDistance < minDistance) {
						minDistance = obstacleDistance;
						potentialObstacleIndex = (int)i;
						potentialObstacleLaneId = obstacleLaneId;
					}
				}
			}

			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];//��ȡǱ���ϰ������Ϣ
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ); //����Ǳ���ϰ�����ٶ�


			SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);//��ȡǱ���ϰ����λ��
			double sObstalce = 0.;//�ϰ���λ��
			double tObstacle = 0.;//�ϰ���ʱ��

			double sMainVehicle = 0.;//����λ��
			double tMainVehicle = 0.;//����ʱ��

			bool isObstalceBehind = false;//�ж��Ƿ��ϰ����Ƿ��������ĺ���

			/*��ȡ�ϰ����������λ�ú�ʱ��*/
			if (!potentialObstacleLaneId.Empty()) {

				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

                isObstalceBehind = !(sMainVehicle >= sObstalce);//�ж��Ƿ��ϰ����Ƿ��������ĺ���
			}
			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
			std::unique_ptr <SimOne_Data_Signal_Lights> plight = std::make_unique<SimOne_Data_Signal_Lights>();

			/*��������*/
			pControl->throttle = 0.12f;
			pControl->brake = 0.f;
			pControl->steering = 0.f;
			pControl->handbrake = false;
			pControl->isManualGear = false;
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
			plight->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_None;

			SSD::SimVector<int> indexOfValidPoints;
			SSD::SimVector<long> roadIdList;
			SimOneAPI::Navigate(inputPoints, indexOfValidPoints, roadIdList);
			HDMapStandalone::MLaneId id(mainVehicleLaneId);
			HDMapStandalone::MSignal light = GetTargetLight(id, mainVehicleLaneId, roadIdList);
			SSD::SimPoint3D stopLine = GetTragetStopLine(light, mainVehicleLaneId);

			SSD::SimPoint2D vehiclePos2D(mainVehiclePos.x, mainVehiclePos.y);
			double safeDistance = UtilMath::distance(vehiclePos2D,SSD::SimPoint2D(stopLine.x, stopLine.y));


			if (case_num == 10)
			{
				if (IsGreenLight(light.id, mainVehicleLaneId, light)) 
				{
					if (mainVehicleSpeed*3.6f > 20) 
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else 
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}
				else 
				{
					if (safeDistance >= 8) 
					{
						if (mainVehicleSpeed*3.6f > 20)
						{
							pControl->throttle = 0.0f;
							pControl->brake = 0.2f;
						}
						else 
						{
							pControl->throttle = 0.4f;
							pControl->brake = 0.0f;
						}
					}
					else if (safeDistance < 8 && safeDistance>1) 
					{
						pControl->throttle = 0.0f;
						pControl->brake = 1.0f;
					}
				}
			
			}

			if (case_num == 11) 
			{
				if (mainVehicleSpeed*3.6f > 20)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
				if (minDistance <= 6.5)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				if (mainVehicleSpeed == 0)
				{
					plight->signalLights = ESimOne_Signal_Light_DoubleFlash;
				}
			}

			if (case_num == 12) 
			{
				if (mainVehicleSpeed*3.6f > 10086) 
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else 
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
				
			}

			if (case_num == 13) 
			{
				SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
				SSD::SimPoint3D newEndPoint(-264, -14.3, 0);

				change_road_function(&inputPoints, newStartPoint, newEndPoint);
				SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				if (sqrt((mainVehiclePos.x + 264)*(mainVehiclePos.x + 264) + (mainVehiclePos.y + 14.3)*(mainVehiclePos.y + 14.3)) < 2.6)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint = targetPointPos;
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				if (mainVehicleSpeed * 3.6 > 10)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 14 || case_num == 15)
			{
				if (mainVehicleSpeed*3.6f > 20) {
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else if(mainVehicleSpeed * 3.6f > 3 && mainVehicleSpeed * 3.6f < 20)
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
				if (minDistance < 15)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.4f;
				}
				else 
				{
				}
			}

			if (case_num == 16)
			{
				SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
				SSD::SimPoint3D newEndPoint(-438, -14, 0);
				// ���� change_road_function ���������µ�·�㼯
				change_road_function(&inputPoints, newStartPoint, newEndPoint);
				SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				if (sqrt((mainVehiclePos.x + 438)*(mainVehiclePos.x + 438) + (mainVehiclePos.y + 14)*(mainVehiclePos.y + 14)) < 2.7)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-422, -14, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}
				
				if (sqrt((mainVehiclePos.x + 422)*(mainVehiclePos.x + 422) + (mainVehiclePos.y + 14)*(mainVehiclePos.y + 14)) < 1.5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-406, -18, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}
				
				if (sqrt((mainVehiclePos.x + 406)*(mainVehiclePos.x + 406) + (mainVehiclePos.y + 18)*(mainVehiclePos.y + 18)) < 1.5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint = targetPointPos;
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				if (sqrt((mainVehiclePos.x + 430)*(mainVehiclePos.x + 430) + (mainVehiclePos.y + 14)*(mainVehiclePos.y + 14)) < 3)
				{
					plight->signalLights = ESimOne_Signal_Light_RightBlinker;
				}

				if (sqrt((mainVehiclePos.x + 456)*(mainVehiclePos.x + 456) + (mainVehiclePos.y + 17)*(mainVehiclePos.y + 17)) < 3)
				{
					plight->signalLights = ESimOne_Signal_Light_LeftBlinker;
				}

				if (mainVehicleSpeed * 3.6 > 20)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 17)
			{
				SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
				SSD::SimPoint3D newEndPoint(-478, -14, 0);

				change_road_function(&inputPoints, newStartPoint, newEndPoint);
				SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				if (sqrt((mainVehiclePos.x + 478)*(mainVehiclePos.x + 478) + (mainVehiclePos.y + 14)*(mainVehiclePos.y + 14)) < 3)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint = targetPointPos;
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				if (mainVehicleSpeed * 3.6 > 30)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 18) 
			{
				count++;

				if (count < 1250)
				{
					pControl->throttle = 0.0f;
				}
				else if (count >= 1250 && count < 1400)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count >= 1400)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}
			}

			if (case_num == 19) 
			{
				if (mainVehicleSpeed*3.6f > 25)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 20) 
			{
				count++;

				if (count < 458)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					goto FINNAL;
				}
				else if (count >= 458)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
			}

			if (case_num == 21) 
			{
				SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
				SSD::SimPoint3D newEndPoint(-215, -14, 0);

				change_road_function(&inputPoints, newStartPoint, newEndPoint);
				SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				if (sqrt((mainVehiclePos.x + 215)*(mainVehiclePos.x + 215) + (mainVehiclePos.y + 14)*(mainVehiclePos.y + 14)) < 2.5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint = targetPointPos;
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				count++;

				if (count < 250)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count >= 250)
				{
					plight->signalLights = ESimOne_Signal_Light_LeftBlinker;
					if (mainVehicleSpeed*3.6f > 25)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}
				
				
			}

			if (case_num == 22) 
			{
				if (mainVehicleSpeed*3.6 >= 20)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.4f;
				}
				else
				{
					pControl->throttle = 0.2f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 23)
			{
				
					if (mainVehicleSpeed > obstacleSpeed)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
			}

			if (case_num == 24) 
			{
				if (mainVehicleSpeed * 3.6 > obstacleSpeed * 3.6-1)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else if (mainVehicleSpeed * 3.6 < 13)
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 25)
			{
				if (mainVehicleSpeed > obstacleSpeed) {
					pControl->throttle = 0.0f;
					pControl->brake = 0.4f;
				}
				else if (mainVehicleSpeed < obstacleSpeed)
				{
					pControl->throttle = 0.2f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 26) 
			{
				SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
				SSD::SimPoint3D newEndPoint(-245, -14, 0);

				change_road_function(&inputPoints, newStartPoint, newEndPoint);
				SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				if (sqrt((mainVehiclePos.x + 245)*(mainVehiclePos.x + 245) + (mainVehiclePos.y + 14)*(mainVehiclePos.y + 14)) < 2.8)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint = targetPointPos;
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}
				if (sqrt((mainVehiclePos.x + 272)*(mainVehiclePos.x + 272) + (mainVehiclePos.y + 17.5)*(mainVehiclePos.y + 17.5)) < 28)
				{
					plight->signalLights = ESimOne_Signal_Light_LeftBlinker;
				}
				else
				{
					plight->signalLights = ESimOne_Signal_Light_None;
				}
				if (mainVehicleSpeed * 3.6 > 20)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 27) 
			{
				SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
				SSD::SimPoint3D newEndPoint(-290, -14, 0);

				change_road_function(&inputPoints, newStartPoint, newEndPoint);
				SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				if (sqrt((mainVehiclePos.x + 290)*(mainVehiclePos.x + 290) + (mainVehiclePos.y + 14)*(mainVehiclePos.y + 14)) < 2.8)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint = targetPointPos;
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}
				if (sqrt((mainVehiclePos.x + 315)*(mainVehiclePos.x + 315) + (mainVehiclePos.y + 17.5)*(mainVehiclePos.y + 17.5)) < 28)
				{
					plight->signalLights = ESimOne_Signal_Light_LeftBlinker;
				}
				else
				{
					plight->signalLights = ESimOne_Signal_Light_None;
				}
				if (mainVehicleSpeed * 3.6 > 20)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 28) 
			{
				count++;

				if (sqrt((mainVehiclePos.x + 283)*(mainVehiclePos.x + 283) + (mainVehiclePos.y + 17.5)*(mainVehiclePos.y + 17.5)) < 28)
				{
					plight->signalLights = ESimOne_Signal_Light_LeftBlinker;
				}
				else
				{
					plight->signalLights = ESimOne_Signal_Light_None;
				}
				if (mainVehicleSpeed * 3.6 > 20)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
				if (count > 850)
				{
					pControl->steering = 0.0f;
					goto FINNAL;
				}
			}

			if (case_num == 29) 
			{
				if (mainVehicleSpeed*3.6f > 64)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
				plight->signalLights = ESimOne_Signal_Light_LeftBlinker;
			}

			if (case_num == 30) 
			{
				count++;

				if (count < 386)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}
				else if (count >= 386 && count <= 600)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count >= 600) 
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}
			}

			if (case_num == 31) {
				if (isObstalceBehind){}
				if (mainVehicleSpeed*3.6f > 20) {
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else {
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			pControl->throttle = 0.0f;
			}

			if (case_num == 32) 
			{
				count++;

				if (count < 858)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}
				else if (count >= 858) 
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
			}

			if (case_num == 33)
			{
				count++;

				if (count < 386)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					goto FINNAL;
				}
				else if (count >= 386 && count <= 1830)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count > 1830)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}

			}

			if (case_num == 34)
			{
				count++;

				if (count < 386)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					goto FINNAL;
				}
				else if (count >= 386 && count <= 650)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count > 650 )
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					if (count > 650)
					{
						pControl->steering = 0.0f;
						goto FINNAL;
					}
				}
			}

			if (case_num == 35)
			{
				count++;

				if (count < 386)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					goto FINNAL;
				}
				else if (count >= 386 && count <= 1700)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count > 1700)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}

			}

			if (case_num == 36)
			{
				count++;

				if (count < 818)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					goto FINNAL;
				}
				else if (count >= 818 && count <= 1950)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count > 1950)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}
			}

			if (case_num == 37)
			{
				count++;

				if (count < 454)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					goto FINNAL;
				}
				else if (count >= 454 && count <= 1600)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count > 1600)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}

			}

			if (case_num == 38)
			{
				count++;

				if (count < 403)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
					goto FINNAL;
				}
				else if (count >= 403 && count <= 600)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 1.0f;
				}
				else if (count > 600)
				{
					if (mainVehicleSpeed*3.6f > 20)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.2f;
					}
					else
					{
						pControl->throttle = 0.4f;
						pControl->brake = 0.0f;
					}
				}

			}

			if (case_num == 39) 
			{
				if (mainVehicleSpeed*3.6f > 20)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
			}

			if (case_num == 40) 
			{
				if (mainVehicleSpeed*3.6f > 30)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}
				pControl->throttle = 0.0f;
			}

			if (case_num == 41) 
			{

				if (sqrt((mainVehiclePos.x + 23)*(mainVehiclePos.x + 23) + (mainVehiclePos.y - 62)*(mainVehiclePos.y - 62)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-24, 50, 0);

					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}
				
				if (sqrt((mainVehiclePos.x + 24)*(mainVehiclePos.x + 24) + (mainVehiclePos.y - 50)*(mainVehiclePos.y - 50)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-23, 46, 0);

					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				if (sqrt((mainVehiclePos.x + 23)*(mainVehiclePos.x + 23) + (mainVehiclePos.y - 46)*(mainVehiclePos.y - 46)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-17, 35, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				if (sqrt((mainVehiclePos.x + 17)*(mainVehiclePos.x + 17) + (mainVehiclePos.y - 35)*(mainVehiclePos.y - 35)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-16, 27, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}


				//ֱ�߹���·
				if (sqrt((mainVehiclePos.x + 11)*(mainVehiclePos.x + 11) + (mainVehiclePos.y - 6)*(mainVehiclePos.y - 6)) < 22)
				{
					count++;
					pControl->steering = 0.25f;
					if (count > 28) 
					{
						pControl->steering = 0.0f;
					}
					goto FINNAL;
				}


				if (sqrt((mainVehiclePos.x + 7)*(mainVehiclePos.x + 7) + (mainVehiclePos.y +18)*(mainVehiclePos.y +18)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(0, -48, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				if (sqrt((mainVehiclePos.x + 0)*(mainVehiclePos.x + 0) + (mainVehiclePos.y + 48)*(mainVehiclePos.y + 48)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(2, -60, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);
				}

				if (sqrt((mainVehiclePos.x - 2)*(mainVehiclePos.x - 2) + (mainVehiclePos.y + 60)*(mainVehiclePos.y + 60)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(1, -76, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 1)*(mainVehiclePos.x - 1) + (mainVehiclePos.y + 76)*(mainVehiclePos.y + 76)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(3.5, -90, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 3.5)*(mainVehiclePos.x - 3.5) + (mainVehiclePos.y + 90)*(mainVehiclePos.y + 90)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(2, -95, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 2)*(mainVehiclePos.x - 2) + (mainVehiclePos.y + 95)*(mainVehiclePos.y + 95)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(2, -110, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 2)*(mainVehiclePos.x - 2) + (mainVehiclePos.y + 110)*(mainVehiclePos.y + 110)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-10, -143, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x + 10)*(mainVehiclePos.x + 10) + (mainVehiclePos.y + 143)*(mainVehiclePos.y + 143)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-42, -153, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x + 42)*(mainVehiclePos.x + 42) + (mainVehiclePos.y + 153)*(mainVehiclePos.y + 153)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-51, -159, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x + 51)*(mainVehiclePos.x + 51) + (mainVehiclePos.y + 159)*(mainVehiclePos.y + 159)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-69, -178, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x + 69)*(mainVehiclePos.x + 69) + (mainVehiclePos.y + 178)*(mainVehiclePos.y + 178)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(-66, -198, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x + 66)*(mainVehiclePos.x + 66) + (mainVehiclePos.y + 198)*(mainVehiclePos.y + 198)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(257, 3, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 257)*(mainVehiclePos.x - 257) + (mainVehiclePos.y - 3)*(mainVehiclePos.y - 3)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(247, 40, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 247)*(mainVehiclePos.x - 247) + (mainVehiclePos.y - 40)*(mainVehiclePos.y - 40)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(238, 67, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 238)*(mainVehiclePos.x - 238) + (mainVehiclePos.y - 67)*(mainVehiclePos.y - 67)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(215, 158, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 215)*(mainVehiclePos.x - 215) + (mainVehiclePos.y - 158)*(mainVehiclePos.y - 158)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(184, 330, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 184)*(mainVehiclePos.x - 184) + (mainVehiclePos.y - 330)*(mainVehiclePos.y - 330)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(177, 390, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 177)*(mainVehiclePos.x - 177) + (mainVehiclePos.y - 390)*(mainVehiclePos.y - 390)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(57, 292, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 57)*(mainVehiclePos.x - 57) + (mainVehiclePos.y - 292)*(mainVehiclePos.y - 292)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(66, 268, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 66)*(mainVehiclePos.x - 66) + (mainVehiclePos.y - 268)*(mainVehiclePos.y - 268)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(125, -27, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 125)*(mainVehiclePos.x - 125) + (mainVehiclePos.y + 27)*(mainVehiclePos.y + 27)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(121, -52, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 121)*(mainVehiclePos.x - 121) + (mainVehiclePos.y + 52)*(mainVehiclePos.y + 52)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(135, -77, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 135)*(mainVehiclePos.x - 135) + (mainVehiclePos.y + 77)*(mainVehiclePos.y + 77)) < 10)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(145, -125, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 145)*(mainVehiclePos.x - 145) + (mainVehiclePos.y + 125)*(mainVehiclePos.y + 125)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint(145, -145, 0);
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				if (sqrt((mainVehiclePos.x - 145)*(mainVehiclePos.x - 145) + (mainVehiclePos.y + 145)*(mainVehiclePos.y + 145)) < 5)
				{
					SSD::SimPoint3D newStartPoint(mainVehiclePos.x, mainVehiclePos.y, 0);
					SSD::SimPoint3D newEndPoint = targetPointPos;
					change_road_function(&inputPoints, newStartPoint, newEndPoint);
					SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath);

				}

				//�������
				if (mainVehicleSpeed * 3.6 > 50)
				{
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
				else
				{
					pControl->throttle = 0.4f;
					pControl->brake = 0.0f;
				}

				if (sqrt((mainVehiclePos.x -6)*(mainVehiclePos.x -6) + (mainVehiclePos.y - 365)*(mainVehiclePos.y - 365)) < 30)
				{
					if (mainVehicleSpeed * 3.6 > 100)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.5f;
					}
					else
					{
						pControl->throttle = 1.0f;
						pControl->brake = 0.0f;
					}
				}

				if (sqrt((mainVehiclePos.x - 46)*(mainVehiclePos.x - 46) + (mainVehiclePos.y + 286)*(mainVehiclePos.y + 286)) < 80)
				{
					
					if (mainVehicleSpeed * 3.6 > 100)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.5f;
					}
					else
					{
						pControl->throttle = 1.0f;
						pControl->brake = 0.0f;
					}
				}

				if (sqrt((mainVehiclePos.x - 300)*(mainVehiclePos.x - 300) + (mainVehiclePos.y + 107)*(mainVehiclePos.y + 107)) < 70)
				{

					if (mainVehicleSpeed * 3.6 > 100)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.5f;
					}
					else
					{
						pControl->throttle = 1.0f;
						pControl->brake = 0.0f;
					}
				}

				if (sqrt((mainVehiclePos.x - 199)*(mainVehiclePos.x - 199) + (mainVehiclePos.y - 257)*(mainVehiclePos.y - 257)) < 70)
				{

					if (mainVehicleSpeed * 3.6 > 100)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.5f;
					}
					else
					{
						pControl->throttle = 1.0f;
						pControl->brake = 0.0f;
					}
				}

				if (sqrt((mainVehiclePos.x - 156)*(mainVehiclePos.x - 156) + (mainVehiclePos.y + 200)*(mainVehiclePos.y + 200)) < 40)
				{

					if (mainVehicleSpeed * 3.6 > 100)
					{
						pControl->throttle = 0.0f;
						pControl->brake = 0.5f;
					}
					else
					{
						pControl->throttle = 1.0f;
						pControl->brake = 0.0f;
					}
				}

			}
			
			double steering = UtilDriver::calculateSteering(targetPath, pGps.get());
			pControl->steering = (float)steering*6.0f;
			FINNAL:
			SimOneAPI::SetSignalLights(MainVehicleId, plight.get());
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());
		}
		
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}
		
		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
