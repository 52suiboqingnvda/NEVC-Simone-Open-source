#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneEvaluationAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilMath.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "hdmap/SampleGetLaneST.h"
#include <memory>
#include <limits>
#include <iostream>

int case_num = 0;

//Main function
//
int main()
{
	bool inAEBState = false;//AEB״̬��־
	bool isSimOneInitialized = false;//SimOneAPI�Ƿ��ѳ�ʼ���ı�־
	const char* MainVehicleId = "0";// ������ID
	bool isJoinTimeLoop = true;// �Ƿ����ʱ��ѭ���ı�־
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop); // ����SimOneAPI�ĳ�ʼ������
	SimOneAPI::SetDriverName(MainVehicleId, "AEB");// ���ü�ʻԱ����Ϊ"AEB"
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);// ���ü�ʻģʽΪAPIģʽ
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);// ʹ�ñ������ݳ�ʼ����������

	int timeout = 20; //���ó�ʱʱ��Ϊ20��
	/*���ص�ͼ*/
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	while (true) {
		int frame = SimOneAPI::Wait();// �ȴ���һ֡

		/*��鰸������״̬*/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		/*��ȡ��������GPS����*/
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>(); 
		if (!SimOneAPI::GetGps(MainVehicleId,pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

       /*��ȡ���������ϰ�����Ϣ*/
		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId,pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}
		/*���SimOne����������״̬*/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);//��ʾ��������λ��
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);//�������������ٶ�

			double minDistance = std::numeric_limits<double>::max();//��ʼ����С����Ϊ������
			int potentialObstacleIndex = pObstacle->obstacleSize;//��ʼ��Ǳ���ϰ��������Ϊ�ϰ�������Ĵ�С������Ч������
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos); //��ȡ���������ڳ�����ID
			SSD::SimString potentialObstacleLaneId = ""; //��ʼ��Ǳ���ϰ������ڳ�����IDΪ���ַ���
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);//��ȡ�ϰ������ά����
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);//��ȡ�ϰ������ڵĳ���ID
				if (mainVehicleLaneId == obstacleLaneId) {//�ж��ϰ����Ƿ������������ڳ���
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);// �������������ϰ���֮���ƽ�����

					if (obstacleDistance < minDistance) {//�жϵ�ǰ�ϰ���ľ����Ƿ�С����֪����С����
						minDistance = obstacleDistance;
						potentialObstacleIndex = (int)i;
						potentialObstacleLaneId = obstacleLaneId;
					}
				}
			}

			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];// ��ȡǱ���ϰ������Ϣ
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);// ����Ǳ���ϰ�����ٶ�


			SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);//��ʾǱ���ϰ���λ��

			// ��ʼ��Ǳ���ϰ����s��tֵ
			double sObstalce = 0;
			double tObstacle = 0;

			// ��ʼ����������s��tֵ
			double sMainVehicle = 0;
			double tMainVehicle = 0;

			bool isObstalceBehind = false;//�ϰ����Ƿ�����������
			if (!potentialObstacleLaneId.Empty()) {//���Ǳ���ϰ���ĳ���ID�Ƿ�Ϊ��

				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);//��ȡǱ���ϰ����ڳ����ϵ�s��tֵ
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);//��ȡ�������ڳ����ϵ�s��tֵ

				isObstalceBehind = !(sMainVehicle >= sObstalce);//�ж�Ǳ���ϰ����Ƿ����������󷽣���Ǳ���ϰ����sֵС����������sֵ��
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();//����һ��SimOne_Data_Control���͵�����ָ��

			//��������
			pControl->throttle = 0.2f;                             //���ſ��ƣ���ΧΪ0��1��0��ʾ��ȫ�ɿ���1��ʾȫ����
			pControl->brake = 0;                                   //ɲ�����ƣ���ΧΪ0��1��0��ʾ��ȫ�ɿ���1��ʾ���ɲ��
			pControl->steering = 0;                                //�����̿��ƣ���ΧΪ-1��1��-1��ʾ���������1��ʾ���Ҵ���
			pControl->handbrake = 0;                               //��ɲ���ƣ�0��ʾδ���£�1��ʾ������ɲ
			pControl->isManualGear = 0;                            //�Ƿ��ֶ�������0��ʾ�Զ�������1��ʾ�ֶ�����
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);    //��λ���ƣ�����ʵ������������Ӧ�ĵ�λ*/ 

			if (mainVehicleSpeed*3.6 >= 34)// �ж������ٶ��Ƿ���ڵ���34km/h
				{
				/*����ٶȴ��ڵ���34km/h����������Ϊ0��ɲ��Ϊ0.2*/
					pControl->throttle = 0.0f;
					pControl->brake = 0.2f;
				}
			else
				{
				/* ����ٶ�С��34km / h����������Ϊ0.3��ɲ��Ϊ0*/
					pControl->throttle = 0.3f;
					pControl->brake = 0.0f;
				}
			

			if (isObstalceBehind) {
				if (minDistance <= 11.5) {//����ϰ������С�ڵ���11.5��
					if (obstacleSpeed >= 2)//����ϰ����ٶȴ��ڵ���2��/��
					{
						case_num = 2;
					}
					else if (case_num == 0 && obstacleSpeed>0 && obstacleSpeed<2)//���case_numΪ0���ϰ����ٶȴ���0��С��2��/��
					{
						case_num = 3;
					}
					else if (case_num == 3 && obstacleSpeed==0)//���case_numΪ3���ϰ����ٶ�Ϊ0
					{
						goto FINNAL;
					}

					pControl->throttle = 0.0f; //��������Ϊ0
					pControl->brake = 1.0f;//����ɲ��Ϊ1
				}
			}

			
			std::cout << "speed:" << mainVehicleSpeed << std::endl;//��������ٶ�
			FINNAL:
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());//���ó���������Ϣ
		}
		else {//���δ��ʼ����ɣ������־��Ϣ
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);//������һ֡
	}
	return 0;
}
