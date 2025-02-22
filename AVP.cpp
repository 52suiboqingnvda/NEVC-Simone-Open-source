#include <iostream>
#include <memory>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>

#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "UtilMath.hpp"
#include "UtilUnit.hpp"
#include "UtilDriver.hpp"
#include "AVPLog.hpp"
#include "AVPPlanner.hpp"
#include "SimOneEvaluationAPI.h"

enum AVPStatus
{
	eApproaching,
	eForwarding,
	eBrakingPrep,
	eReversing,
	eFinishing,
	eWaiting,
	eLeaving,
	eEnded,
	eUnknown
};

void FindTargetParkingSpace(const SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaces,
	const SimOne_Data_Obstacle* obstacles, HDMapStandalone::MParkingSpace& targetParkingSpace)
{
	
void RearrangeKnots(HDMapStandalone::MParkingSpace& space)
{
	size_t knotSize = space.boundaryKnots.size();
	SSD::SimPoint2D heading{ space.heading.x, space.heading.y };
	SSD::SimPoint3DVector arrangedBoundaryKnots(knotSize);
	double headingErrorThresholdDegree = 10.;
	for (size_t i = 0; i < knotSize; ++i)
	{
		SSD::SimPoint2D dir = { space.boundaryKnots[i + 1].x - space.boundaryKnots[i].x,
								space.boundaryKnots[i + 1].y - space.boundaryKnots[i].y };
		double angle = UtilMath::Angle(heading, dir);
		if (angle > 0. && angle < UtilUnit::DegreeToRad(headingErrorThresholdDegree))
		{
			for (size_t j = 0; j < knotSize; ++j)
			{
				size_t index = i + j + 2 < knotSize ? i + j + 2 : i + j + 2 - knotSize;
				arrangedBoundaryKnots[j] = space.boundaryKnots[index];
			}
			break;
		}
	}
	for (size_t i = 0; i < knotSize; ++i)
	{
		space.boundaryKnots[i] = arrangedBoundaryKnots[i];
	}
}

bool InParkingSpace(const SSD::SimPoint3D& pos, const double theta, const double length, const double width, const HDMapStandalone::MParkingSpace& space)
{
	SSD::SimPoint3D p1(pos.x + width / 2. * cos(theta) + length / 2. * sin(theta),
		pos.y + width / 2. * sin(theta) - length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p2(pos.x + width / 2. * cos(theta) - length / 2. * sin(theta),
		pos.y + width / 2. * sin(theta) + length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p3(pos.x - width / 2. * cos(theta) - length / 2. * sin(theta),
		pos.y - width / 2. * sin(theta) + length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p4(pos.x - width / 2. * cos(theta) + length / 2. * sin(theta),
		pos.y - width / 2. * sin(theta) - length / 2. * cos(theta), pos.z);

	return UtilMath::InRectangleMargin(p1, space.boundaryKnots[0], space.boundaryKnots[2]) || UtilMath::InRectangleMargin(p2, space.boundaryKnots[0], space.boundaryKnots[2]) ||
		UtilMath::InRectangleMargin(p3, space.boundaryKnots[0], space.boundaryKnots[2]) || UtilMath::InRectangleMargin(p4, space.boundaryKnots[0], space.boundaryKnots[2]);
}

int main(int argc, char* argv[])
{
	std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
	std::unique_ptr<SimOne_Data_Obstacle> obstaclesPtr = std::make_unique<SimOne_Data_Obstacle>();
	std::unique_ptr<SimOne_Data_WayPoints> wayPointsPtr = std::make_unique<SimOne_Data_WayPoints>();


	bool leaveAfterParked = true;
	bool isJoinTimeLoop = true;
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);

	SimOneAPI::SetDriverName(0, "AVP");
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);// 使用本地数据初始化评估服务

	int timeoutSecond = 20;
	if (!SimOneAPI::LoadHDMap(timeoutSecond))
	{
		std::cout << "Failed to load map!" << std::endl;
		return 0;
	}


	SSD::SimPoint3D startPoint;
	if (SimOneAPI::GetGps(0, gpsPtr.get()))
	{
		startPoint.x = gpsPtr->posX;
		startPoint.y = gpsPtr->posY;
		startPoint.z = gpsPtr->posZ;
	}
	else
	{
		std::cout << "Fetch GPS failed" << std::endl;
	}

	SimOneAPI::GetGroundTruth(MainVehicleId, obstaclesPtr.get());
	for (size_t i = 0; i < obstaclesPtr->obstacleSize; ++i)
	{
		std::cout << "Obstacles[" << i << "]: " << std::endl;
		auto& obstacle = obstaclesPtr->obstacle[i];
		std::cout << "[" << obstacle.posX << ", " << obstacle.posY << "]" << std::endl;
	}
	std::cout << std::endl;

	while (!SimOneAPI::GetWayPoints(MainVehicleId, wayPointsPtr.get()))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	auto lastWayPoint = wayPointsPtr->wayPoints[wayPointsPtr->wayPointsSize - 1];
	SSD::SimPoint3D terminalPoint{ lastWayPoint.posX, lastWayPoint.posY, 0. };

	VehicleParam veh;
	veh.Lr = 0.88;
	veh.Lf = 1.;
	veh.L = 2.9187;

	AVPStatus status = AVPStatus::eForwarding;

	bool brakingPrepStarted = false;
	bool reversingStarted = false;
	bool finishingStarted = false;
	bool waitingStarted = false;
	double brakingPrepTime = 1.;
	double waitingTimeSecond = 3.;
	auto startTime = std::chrono::system_clock::now();
	double headingErrorThresholdDegree = 1.;
	double parkingEndDistanceThreshold = 0.5;
	double leavingEndDistanceThreshold = 0.5;

	while (1)
	{
		int frame = SimOneAPI::Wait();//获取当前帧数据

		/*检查案例运行状态*/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		if (!SimOneAPI::GetGps(MainVehicleId, gpsPtr.get()))//获取主车的 GPS 数据
		{
			std::cout << "Fetch GPS failed" << std::endl;;//如果获取 GPS 数据失败，则输出错误信息
		}

#if AVP_LOG
		AVPLog::getInstance().addData(gpsPtr->posX);
		AVPLog::getInstance().addData(gpsPtr->posY);
#endif // AVP_LOG

		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();//创建一个SimOne_Data_Control类型的智能指针

		// Control mainVehicle without SimOneDriver 不使用SimOneDriver控制主车辆
		pControl->throttle = 1.0f;                             //油门控制，范围为0到1，0表示完全松开，1表示全油门
		pControl->brake = 0;                                   //刹车控制，范围为0到1，0表示完全松开，1表示最大刹车
		pControl->steering = 0;                                //方向盘控制，范围为-1到1，-1表示向左打满，1表示向右打满
		pControl->handbrake = 0;                               //手刹控制，0表示未踩下，1表示踩下手刹
		pControl->isManualGear = 0;                            //是否手动换挡，0表示自动换挡，1表示手动换挡
		pControl->gear = static_cast<ESimOne_Gear_Mode>(1);    //档位控制，根据实际需求设置相应的档位*/ 
		double speedKmH = UtilUnit::MsToKmH(UtilMath::Length({ gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ }));//计算当前速度（单位：千米/小时）
		if (speedKmH*3.6<= 10086)
		{
			pControl->throttle = 1.0f;;//设置油门值为0.03
			pControl->brake = 0.0f;//刹车值设为0
		}
		std::cout << "speed:" << speedKmH << std::endl;//输出主车速度

		SimOneAPI::SetDrive(MainVehicleId, pControl.get());//设置车辆控制信息
		

		SimOneAPI::NextFrame(frame);
	}

	return 0;
}
