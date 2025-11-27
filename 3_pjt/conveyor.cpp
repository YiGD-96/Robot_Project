#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class ConveyorBeltPlugin : public ModelPlugin
  {
  public:
    ConveyorBeltPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->world = _model->GetWorld();

      if (_sdf->HasElement("belt_velocity"))
      {
        this->beltVelocity = _sdf->Get<double>("belt_velocity");
      }
      else
      {
        this->beltVelocity = 0.5; // 기본 속도
      }

      // ✅ 벨트 표면 가져오기
      if (_sdf->HasElement("belt_link"))
      {
        std::string beltLinkName = _sdf->Get<std::string>("belt_link");
        this->beltLink = _model->GetLink(beltLinkName);
      }

      if (!this->beltLink)
      {
        gzerr << "❌ belt_surface 링크를 찾을 수 없습니다! SDF 파일을 확인하세요." << std::endl;
        return;
      }

      // ✅ 벨트 크기 가져오기
      if (_sdf->HasElement("belt_length"))
      {
        this->beltLength = _sdf->Get<double>("belt_length");
      }
      else
      {
        this->beltLength = 2.0; // 기본값
      }

      if (_sdf->HasElement("belt_width"))
      {
        this->beltWidth = _sdf->Get<double>("belt_width");
      }
      else
      {
        this->beltWidth = 1.0; // 기본값
      }

      // ✅ 물리 업데이트 이벤트 등록
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ConveyorBeltPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      if (!this->beltLink) return;

      ignition::math::Pose3d beltPose = this->beltLink->WorldPose();

      // ✅ 벨트 위의 모든 모델 가져오기 (벨트 영역 내에 있는 물체만)
      for (auto &model : this->world->Models())
      {
        if (model->GetName() != this->model->GetName()) // 컨베이어 벨트 자체는 제외
        {
          // 모델의 현재 위치 가져오기
          ignition::math::Pose3d pose = model->WorldPose();

          // ✅ 벨트 위에 있는지 확인 (X, Y 범위 내)
          double xMin = beltPose.Pos().X() - (this->beltLength / 2.0);
          double xMax = beltPose.Pos().X() + (this->beltLength / 2.0) + 0.04; // ✅ 벨트 끝에서 0.04m 더 이동
          double yMin = beltPose.Pos().Y() - (this->beltWidth / 2.0);
          double yMax = beltPose.Pos().Y() + (this->beltWidth / 2.0);

          bool isOnBelt = (pose.Pos().X() >= xMin && pose.Pos().X() <= xMax) &&
                          (pose.Pos().Y() >= yMin && pose.Pos().Y() <= yMax) &&
                          (pose.Pos().Z() > beltPose.Pos().Z()); // 벨트보다 위에 있는 경우

          if (isOnBelt)
          {
            // X축 방향으로 물체 이동
            pose.Pos().X() += this->beltVelocity * this->world->Physics()->GetMaxStepSize();
            model->SetWorldPose(pose);
          }
        }
      }
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::LinkPtr beltLink; // ✅ 벨트 표면 링크 저장
    event::ConnectionPtr updateConnection;
    double beltVelocity;
    double beltLength; // ✅ 벨트 길이
    double beltWidth;  // ✅ 벨트 너비
  };

  GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)
}

