//////////////////////////////////////////////////////////////////////////
// Animation.cpp -- Source file for useful Classes for character animation
//
// Liming Zhao
// 11/02/2007
// 

#include "StdAfx.h"  // Only needed in MFC

#include "Animation.h"
#include "pinv.h"
#include <Eigen/LU>
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Vector4d;


//////////////////////////////////////////////////////////////////////////
//
// Joint
//
//////////////////////////////////////////////////////////////////////////

// Constructors
Joint::Joint()
{
	m_name = "";
	m_id = -1;
	m_pParent = NULL;
	m_pData = NULL;
}

Joint::Joint(const int& id, const string& name)
{
	m_id = id;
	m_name = name;
	m_pParent = NULL;
	m_pData = NULL;
}

Joint::Joint(const string& name)
{
	m_name = name;
	m_pParent = NULL;
	m_pData = NULL;
}

// Destructor
Joint::~Joint()
{
}

// Member functions
Joint* Joint::GetParent()
{
	return m_pParent;
}

unsigned int Joint::GetChildCount() const
{
	return unsigned int (m_children.size());
}

Joint* Joint::GetChildAt(unsigned int index)
{
	return m_children[index];
}

void Joint::UpdateTransformation(bool bRecursive)
{
	Transform parentTransform;
	if (m_pParent)
	{
		parentTransform = m_pParent->GetGlobalTransform();
	}

	m_global = parentTransform * m_local;

	if (bRecursive)
	{
		vector<Joint*>::const_iterator iter;
		for (iter = m_children.begin(); iter != m_children.end(); iter++)
		{
			(*iter)->UpdateTransformation(true);
		}
	}
}

void Joint::SetName(const string& name)
{
	m_name = name;
}

void Joint::SetID(const int& id)
{
	m_id = id;
}

void Joint::SetChannelCount(const unsigned int& count)
{
	m_channelCount = count;
}

void Joint::SetLocalTransform(const Transform& transform)
{
	m_local = transform;
}

void Joint::SetLocalTranslation(const vec3& translation)
{
	m_local.m_translation = translation;
}

void Joint::SetLocalRotation(const mat3& rotation)
{
	m_local.m_rotation = rotation;
}

void Joint::SetData(void* pData)
{
	m_pData = pData;
}

const string& Joint::GetName() const
{
	return m_name;
}

const int& Joint::GetID() const
{
	return m_id;
}

const unsigned int& Joint::GetChannelCount() const
{
	return m_channelCount;
}

const Transform& Joint::GetLocalTransform() const
{
	return m_local;
}

const vec3& Joint::GetLocalTranslation() const
{
	return m_local.m_translation;
}

const mat3& Joint::GetLocalRotation() const
{
	return m_local.m_rotation;
}

const Transform& Joint::GetGlobalTransform() const
{
	return m_global;
}

const vec3& Joint::GetGlobalTranslation() const
{
	return m_global.m_translation;
}

const mat3& Joint::GetGlobalRotation() const
{
	return m_global.m_rotation;
}

void* Joint::GetData()
{
	return m_pData;
}

void Joint::AttachJoints(Joint* pParent, Joint* pChild)
{
	if (pChild)
	{
		Joint* pOldParent = pChild->m_pParent;
		if (pOldParent)
		{
			// erase the child from old parent's children list
			vector<Joint*>::iterator iter;
			for (iter = pOldParent->m_children.begin(); iter != pOldParent->m_children.end(); iter++)
			{
				if ((*iter) == pChild)
				{
					iter = pOldParent->m_children.erase(iter);
				}
			}
		}
		// Set the new parent
		pChild->m_pParent = pParent;
		// Add child to new parent's children list
		if (pParent)
		{
			pParent->m_children.push_back(pChild);
		}
	}
}

void Joint::DetachJoints(Joint* pParent, Joint* pChild)
{
	if (pChild && pChild->m_pParent == pParent)
	{
		if (pParent)
		{
			// erase the child from parent's children list
			vector<Joint*>::iterator iter;
			for (iter = pParent->m_children.begin(); iter != pParent->m_children.end(); iter++)
			{
				if ((*iter) == pChild)
				{
					iter = pParent->m_children.erase(iter);
				}
			}
		}
		pChild->m_pParent = NULL;
	}
}

//////////////////////////////////////////////////////////////////////////
//
// Skeleton
//
//////////////////////////////////////////////////////////////////////////

Skeleton::Skeleton()
{
	m_pRoot = NULL;
	m_jointCount = 0;
	m_selectedJoint = -1;
}

Skeleton::~Skeleton()
{	
	m_ikChain.clear();
	for(vector<Joint*>::iterator iter = m_joints.begin(); iter != m_joints.end(); iter++)
	{
		delete (*iter);
	}
	m_joints.clear();
	
}

bool Skeleton::LoadFromFile(ifstream& inFile)
{
	if (m_joints.size() > 0)
	{
		m_ikChain.clear();
		for(vector<Joint*>::iterator iter = m_joints.begin(); iter != m_joints.end(); iter++)
		{
			delete (*iter);
		}
		m_joints.clear();
		m_jointCount = 0;
		m_pRoot = NULL;
		m_selectedJoint = -1;
		m_goalPosition = vec3Zero;
	}

	string readString, jointname;
	unsigned int channelCount;
		
	inFile >> readString;
	if (readString != "HIERARCHY")
		return false;
	inFile >> readString;
	if (readString != "ROOT")
		return false;
	inFile.get(); //" "
	getline(inFile, jointname);// joint name
	Joint* joint = new Joint(jointname);
	m_joints.push_back(joint);
	joint->SetID(unsigned(m_joints.size() - 1));
	joint->SetChannelCount(6);
	m_pRoot = joint;
	inFile >> readString; // "{"
	inFile >> readString; // "OFFSET"
	getline(inFile, readString);	// Root translation values
	inFile >> readString;
	if (readString != "CHANNELS")
		return false;
	inFile >> channelCount;
	joint->SetChannelCount(channelCount);
	getline(inFile, readString);	// " Xposition Yposition Zposition Zrotation Xrotation Yrotation"
	inFile >> readString;
	while(readString != "}") 
	{		
		if (!LoadFromFileRec(inFile, joint, readString))
		{
			return false;
		}
		inFile >> readString;
	}
	if (readString != "}")
		return false;

	m_jointCount = unsigned int(m_joints.size());
	return true;
}

bool Skeleton::LoadFromFileRec(ifstream &inFile, Joint *pParent, string prefix)
{
	string readString, jointname;
	vec3 offsets;
	unsigned int channelCount;
	if (prefix == "JOINT")
	{
		inFile.get(); //" "
		getline(inFile, jointname);// joint name
		Joint* joint = new Joint(jointname);
		m_joints.push_back(joint);
		joint->SetID(unsigned(m_joints.size() - 1));	
		Joint::AttachJoints(pParent, joint);
		inFile >> readString; // "{"
		inFile >> readString; // "OFFSET"
		inFile >> offsets[0] >> offsets[1] >> offsets[2];
		joint->SetLocalTranslation(offsets);
		inFile >> readString; // "CHANNELS"
		inFile >> channelCount;
		joint->SetChannelCount(channelCount);
		getline(inFile, readString);// " Zrotation Xrotation Yrotation"
		inFile >> readString; // "Joint" or "}" or "End"
		while (readString != "}")
		{
			if (LoadFromFileRec(inFile, joint, readString) == false)
				return false;
			inFile >> readString; // "Joint" or "}" or "End"
		}
		return true;
	}else if (prefix == "End")
	{	
		inFile.get(); //" "
		getline(inFile, jointname);// joint name
		Joint* joint = new Joint(jointname);
		m_joints.push_back(joint);
		joint->SetID(unsigned(m_joints.size() - 1));
		joint->SetChannelCount(0);
		Joint::AttachJoints(pParent, joint);
		inFile >> readString; // "{"
		inFile >> readString; // "OFFSET"
		inFile >> offsets[0] >> offsets[1] >> offsets[2];
		joint->SetLocalTranslation(offsets);
		inFile >> readString; // "}"
		return true;
	}else
		return false;
}

void Skeleton::SaveToFile(ofstream& outFile)
{
	outFile << "HIERARCHY" << endl;
	outFile << "ROOT " << m_pRoot->GetName() << endl;
	outFile << "{" << endl;
	outFile << "\tOFFSET 0.00 0.00 0.00" << endl;
	outFile << "\tCHANNELS " << m_pRoot->GetChannelCount() << " Xposition Yposition Zposition Zrotation Xrotation Yrotation" << endl;
	unsigned int childCount = m_pRoot->GetChildCount();
	for (unsigned int i = 0; i < childCount; i++)
	{
		Joint* pChild = m_pRoot->GetChildAt(i);
		SaveToFileRec(outFile, pChild, 1);
	}
	outFile << "}" << endl;
}

void Skeleton::SaveToFileRec(ofstream& outFile, Joint* pJoint, unsigned int level)
{
	string indentation = "";
	for (unsigned int i = 0; i < level; i++)
	{
		indentation += "\t";
	}

	if (pJoint->GetChannelCount() == 3)
	{
		outFile << indentation << "JOINT " << pJoint->GetName() << endl;
		outFile << indentation << "{" << endl;
		const vec3& offsets = pJoint->GetLocalTranslation();
		outFile << indentation << "\tOFFSET " << offsets[0] << " " << offsets[1] << " " << offsets[2] << endl;
		outFile << indentation << "\tCHANNELS " << pJoint->GetChannelCount() << " Zrotation Xrotation Yrotation" << endl;
	}else
	{
		outFile << indentation << "End " << pJoint->GetName() << endl;
		outFile << indentation << "{" << endl;
		const vec3& offsets = pJoint->GetLocalTranslation();
		outFile << indentation << "\tOFFSET " << offsets[0] << " " << offsets[1] << " " << offsets[2] << endl;		
	}
	unsigned int childCount = pJoint->GetChildCount();
	for (unsigned int i = 0; i < childCount; i++)
	{
		Joint* pChild = pJoint->GetChildAt(i);
		SaveToFileRec(outFile, pChild, level + 1);
	}
	outFile << indentation << "}" << endl;
}

Joint* Skeleton::GetJointByName(const string& name)
{
	vector<Joint*>::const_iterator iter;
	for (iter = m_joints.begin(); iter != m_joints.end(); iter++)
	{
		if (name == ((*iter)->GetName()))
			return (*iter);
	}
	return NULL;
}

Joint* Skeleton::GetJointByID(const unsigned int& id)
{
	if (id >= 0 && id < m_jointCount)
	{
		return m_joints[id];
	}else
		return NULL;
}

Joint* Skeleton::GetRootJoint()
{
	return m_pRoot;
}

void Skeleton::UpdateFK(Joint* pRoot)
{
	if (pRoot == NULL)
	{
		pRoot = m_pRoot;
	}
	pRoot->UpdateTransformation(true);
}

void Skeleton::ReadFromFrame(Frame* pFrame)
{
	if (m_jointCount != pFrame->GetJointCount())
		return;
	m_pRoot->SetLocalTranslation(pFrame->m_rootTranslation);
	m_pRoot->SetLocalRotation(pFrame->m_rotationData[m_pRoot->GetID()]);
	unsigned int totalChildren = m_pRoot->GetChildCount();
	for (unsigned int i = 0; i < totalChildren; i++)
	{
		ReadFromFrameRec(m_pRoot->GetChildAt(i), pFrame->m_rotationData);
	}
	UpdateFK(m_pRoot);
}

void Skeleton::ReadFromFrameRec(Joint* pJoint, mat3* pRotationData)
{
	pJoint->SetLocalRotation(pRotationData[pJoint->GetID()]);
	unsigned int totalChildren = pJoint->GetChildCount();
	for (unsigned int i = 0; i < totalChildren; i++)
	{
		ReadFromFrameRec(pJoint->GetChildAt(i), pRotationData);
	}
}

void Skeleton::WriteToFrame(Frame* pFrame)
{
	if (m_jointCount != pFrame->GetJointCount())
		return;
	pFrame->m_rootTranslation = m_pRoot->GetLocalTranslation();
	pFrame->SetJointRotation(m_pRoot->GetID(), m_pRoot->GetLocalRotation());
	unsigned int totalChildren = m_pRoot->GetChildCount();
	for (unsigned int i = 0; i < totalChildren; i++)
	{
		WriteToFrameRec(m_pRoot->GetChildAt(i), pFrame);
	}
}

void Skeleton::WriteToFrameRec(Joint* pJoint, Frame* pFrame)
{
	pFrame->SetJointRotation(pJoint->GetID(), pJoint->GetLocalRotation());
	unsigned int totalChildren = pJoint->GetChildCount();
	for (unsigned int i = 0; i < totalChildren; i++)
	{
		WriteToFrameRec(pJoint->GetChildAt(i), pFrame);
	}
}

const vec3& Skeleton::GetGoalPosition() const
{
	return m_goalPosition;
}

void Skeleton::SetGoalPosition(const vec3& pos)
{
	m_goalPosition = pos;
}

void Skeleton::SetSelectedJoint(const int& selectedJoint)
{
	m_selectedJoint = selectedJoint;
	m_ikChain.clear();
	if (m_selectedJoint >= 0 && m_selectedJoint < int(m_jointCount))
	{		
		Joint* tmp = m_joints[selectedJoint];
		m_goalPosition = tmp->GetGlobalTranslation();
		if (tmp != NULL && tmp->GetParent() != NULL)
		{
			// terminate at the joint before root joint, so that root will not change during IK	
			tmp = tmp->GetParent();
			while(tmp->GetParent() != NULL)
			{
				m_ikChain.push_back(tmp);
				tmp = tmp->GetParent();
			}
		}
	}
	
}

//////////////////////////////////////////////////////////////////////////
// Solve Inverse Kinematics using CCD approach
// When this functions is called, the following are given
// IK chain from selected Joint to root is store in m_ikChain.
//        However, the first Joint is the parent of the selected Joint, the last Joint is the child of m_pRoot.
//        Basically you are operating the IK chain without moving the Skeleton root.
// End-effector(the selected Joint) is given as Joint* pEndEffector = m_joints[m_selectedJoint];
// Goal position is in m_goalPosition in world coordinates
// Maximum iteration is defined in MAX_ITER
// Please follow the structure of given code
void Skeleton::SolveIKCCD()
{
    if (m_ikChain.size() > 0) // If there is at east some Joints in the IK chain for manipulation
    {
        Joint* pEndEffector = m_joints[m_selectedJoint]; // Get the end-effector
        vec3 dist = m_goalPosition - pEndEffector->GetGlobalTranslation(); // Compute distance error		
        if (dist.Length() > EPSILONDIST) // There is need for CCD
        {
            // You might want to do some preprocessing here

			vector<float> weights(m_ikChain.size());

			for (unsigned int i = 0, chainSize = m_ikChain.size(); i < chainSize; ++i)
				weights[i] = 0.1f + (1.0f - float(i) / float(chainSize)) * 0.2f;

            for (unsigned int i = 0; i < MAX_ITER; i++)
            {
				int joint_index = -1;
				for (auto pJoint : m_ikChain)
                {
					// Let pJoint be the current Joint on the IK chain for handling                    
					++joint_index;                    

                    // Compute the rotation axis and angle to minimize the error
                    // You need to handle the co-linear case to avoid invalid vec3 cross product
                    // Add your code here

					auto vc = (pEndEffector->GetGlobalTranslation() - pJoint->GetGlobalTranslation()).Normalize();
					auto vg = (m_goalPosition - pJoint->GetGlobalTranslation()).Normalize();
					auto rr = pJoint->GetGlobalRotation().Transpose();
					auto dot = vc * vg;
					auto colinear = fabs(fabs(dot) - 1) < EPSILON*10;

					if (colinear) continue;

					auto axis = rr * vc.Cross(vg);
					auto angle = acos(dot) * weights[joint_index];

                    // Update rotation at pJoint
                    // Update FK from pJoint
                    // Add your code
					auto new_local_rotation = mat3::Identity();

					if (angle > 10 * EPSILON)					
						new_local_rotation = mat3::Rotation3DRad(axis, angle);

					pJoint->SetLocalRotation(new_local_rotation * pJoint->GetLocalRotation());
					pJoint->UpdateTransformation(true);

                    // Check error, if it is close enough, terminate the CCD iteration
                    dist = m_goalPosition - pEndEffector->GetGlobalTranslation();
					
                    if (dist.Length() < EPSILONDIST)
                        return;
                }
            }
        }
    }
} 

void Skeleton::SolveIKJacobian(bool bPseudoInverse)
{
    // Add your code here

    // If there is at east some Joints in the IK chain for manipulation
    if (m_ikChain.size() <= 0) return;

    auto pEndEffector = m_joints[m_selectedJoint]; // Get the end-effector
    auto dist = m_goalPosition - pEndEffector->GetGlobalTranslation(); // Compute distance error		
    if (dist.Length() <= EPSILONDIST) return; // There is no need for IK

    // You might want to do some preprocessing here

    const auto chainSize = m_ikChain.size();

    auto thetas = vector<vec3>(chainSize);

    auto J = Eigen::MatrixXd(3, 3 * chainSize);

    const auto h = 0.01f;

    // init rotation parameters
    for (unsigned int i = 0; i < chainSize; i++)
    {
        auto joint = m_ikChain[i];
        auto axis = vec3();
        auto angle = 0.f;
        joint->GetLocalRotation().ToQuaternion().ToAxisAngle(axis, angle);
        if (fabs(axis.Length()) < EPSILON)
        {
            thetas[i] = vec3(0, 0, 0);
        }
        else
        {
            thetas[i] = axis.Normalize() * angle;
        }        
    }

    for (unsigned int i = 0; i < MAX_ITER; i++)
    {
        int joint_index = -1;

        auto p = pEndEffector->GetGlobalTranslation();

        J.Zero(J.rows(), J.cols());

        for (auto pJoint : m_ikChain)
        {
            // Let pJoint be the current Joint on the IK chain for handling                    
            ++joint_index;

            auto org_rot = pJoint->GetLocalRotation();

            // 3 components
            for (int j = 0; j < 3; j++)
            {
                // get the rotation parameter theta
                auto theta = vec3(thetas[joint_index]);

                // update the rotation parameter
                theta[j] += h;

                auto angle = theta.Length();                

                auto axis = angle < EPSILON ? vec3(0,0,0) : theta.Normalize();

                auto q = Quaternion();

                q.FromAxisAngle(axis, angle);

                auto new_rot = q.ToRotation();

                // update the rotation angle
                pJoint->SetLocalRotation(new_rot);
                pJoint->UpdateTransformation(true);
                // new postition of the end effector
                auto pp = pEndEffector->GetGlobalTranslation();

                // partial derivative
                auto pet = (pp - p) / h;

                // update the jacobian matrix
                J(0, joint_index * 3 + j) = pet[0];
                J(1, joint_index * 3 + j) = pet[1];
                J(2, joint_index * 3 + j) = pet[2];

                // roll back
                pJoint->SetLocalRotation(org_rot);
                pJoint->UpdateTransformation(true);
            }
        }

        const auto e = m_goalPosition - p;
        auto E = Eigen::Vector3d(e[0],e[1],e[2]);        

        MatrixXd JP = bPseudoInverse ? pinv(J) : J.transpose();               

        // Nx1 = Nx3 * 3x1        
        MatrixXd delta_thetas = JP * E;
        auto rows = delta_thetas.rows();
        auto cols = delta_thetas.cols();
        auto alpha = 1.0;

        auto back_thetas = thetas;
        auto max_iter = 20;

        // find a good alpha
        while (max_iter--)
        {
            // update the rotation paramters and rotation matrices
            for (unsigned int i = 0; i < chainSize; i++)
            {
                auto pJoint = m_ikChain[i];
                auto& theta = thetas[i];               

                for (int j = 0; j < 3; j++)
                {
                    auto delta_theta = delta_thetas(i * 3 + j, 0);
                    theta[j] += alpha*delta_theta;
                }

                auto angle = theta.Length();

                auto axis = angle < EPSILON ? vec3(0, 0, 0) : vec3(theta).Normalize();

                auto q = Quaternion();

                q.FromAxisAngle(axis, angle);

                auto new_rot = q.ToRotation();

                // update the rotation angle
                pJoint->SetLocalRotation(new_rot);
                pJoint->UpdateTransformation(true);
            }

            auto org_err = e.Length();
            auto now_err = (m_goalPosition - pEndEffector->GetGlobalTranslation()).Length();

            if (now_err < org_err) {
                break;
            }
            
            alpha *= 0.5;
            // rollback
            thetas = back_thetas;
        }

        // Check error, if it is close enough, terminate the CCD iteration        
        if ((m_goalPosition - pEndEffector->GetGlobalTranslation()).Length() < EPSILONDIST)
            return;
    }
}

void Skeleton::SolveIKJacobianPseudoInverse()
{
	// Add your code here
    this->SolveIKJacobian(true);
}

void Skeleton::SolveIKJacobianTranspose()
{
    this->SolveIKJacobian(false);
}



//////////////////////////////////////////////////////////////////////////
//
// Frame
//
//////////////////////////////////////////////////////////////////////////
Frame::Frame()
{
	m_jointCount = 0;
	m_rotationData = NULL;
	m_quaternionData = NULL;
}

Frame::~Frame()
{
	if (m_rotationData)
		delete[] m_rotationData;
	if (m_quaternionData)
	{
		delete[] m_quaternionData;
	}
}

void Frame::SetJointCount(unsigned int jointCount)
{
	if (m_jointCount != jointCount)
	{
		m_jointCount = jointCount;
		if (m_rotationData)
		{
			delete[] m_rotationData;
		}
		m_rotationData = new mat3[jointCount];
		if (m_quaternionData)
		{
			delete[] m_quaternionData;
		}
		m_quaternionData = new Quaternion[jointCount];
	}
}

const unsigned int& Frame::GetJointCount() const
{
	return m_jointCount;
}

void Frame::LoadFromFile(ifstream& inFile, Skeleton* pSkeleton)
{
	float tx, ty, tz, ry, rx, rz;
	inFile >> tx >> ty >> tz;
	m_rootTranslation = vec3(tx, ty, tz);
	for (unsigned int i = 0; i < m_jointCount; i++)
	{
		Joint* pJoint = pSkeleton->GetJointByID(i);
		if (pJoint->GetChannelCount() > 0)
		{
			inFile >> rz >> rx >> ry;
		}else
		{
			rz = rx = ry = 0.0f;
		}
		m_rotationData[i].FromEulerAnglesZXY(vec3(rx, ry, rz) * Deg2Rad);
		m_quaternionData[i].FromRotation(m_rotationData[i]);
	}
}

void Frame::SaveToFile(ofstream& outFile, Skeleton* pSkeleton)
{
	outFile << setprecision(6);
	outFile << m_rootTranslation[0] << "\t" << m_rootTranslation[1] << "\t" << m_rootTranslation[2];
	vec3 angles;
	for (unsigned int i = 0; i < m_jointCount; i++)
	{
		Joint* pJoint = pSkeleton->GetJointByID(i);
		if (pJoint->GetChannelCount() > 0)
		{
			m_rotationData[i].ToEulerAnglesZXY(angles);
			angles *= Rad2Deg;
			outFile << "\t" << angles[VZ] << "\t" << angles[VX] << "\t" << angles[VY];
		}
	}
	outFile << endl;
}

void Frame::Clone(const Frame& frameData)
{
	SetJointCount(frameData.m_jointCount);
	m_rootTranslation = frameData.m_rootTranslation;
	for (unsigned int i = 0; i < m_jointCount; i++)
	{
		m_rotationData[i] = frameData.m_rotationData[i];
		m_quaternionData[i] = frameData.m_quaternionData[i];
	}
}

void Frame::SetRootTranslation(const vec3& translation)
{
	m_rootTranslation = translation;
}

void Frame::SetJointRotation(const unsigned int& index, const mat3& rotation)
{
	assert(index >= 0 && index < m_jointCount);
	m_rotationData[index] = rotation;
	m_quaternionData[index].FromRotation(rotation);
}

void Frame::SetJointRotation(const unsigned int& index, const Quaternion& rotation)
{
	assert(index >= 0 && index < m_jointCount);	
	m_quaternionData[index] = rotation;
	m_rotationData[index].FromQuaternion(rotation);
}
//////////////////////////////////////////////////////////////////////////
//
vec3 CubicVec3(vec3 &d1, vec3 &d2, vec3 &d3, vec3 &d4, float t)
{
	vec3 a = d2; 
	vec3 b = d2 - d1; 
	vec3 c = (d3 - d2) * 3 - (d2 - d1) * 2 - (d4 - d3); 
	vec3 d = (d2 - d3) * 2 + d2 - d1 + d4 - d3;
	return a + b * t + c * t * t + d * t * t * t;
}


void Frame::Cubic(const Frame& frame0, const Frame& frame1, const Frame& frame2, const Frame& frame3, Frame& targetFrame, float fPerc)
{
	targetFrame.m_rootTranslation = frame1.m_rootTranslation * (1.0f - fPerc) + frame2.m_rootTranslation * fPerc;
	for (unsigned int i = 0; i < frame0.m_jointCount; i++)
	{
		targetFrame.m_rotationData[i].FromQuaternion(targetFrame.m_quaternionData[i]);
		const mat3& rot0 = frame0.GetJointRotation(i);
		const mat3& rot1 = frame1.GetJointRotation(i);
		const mat3& rot2 = frame2.GetJointRotation(i);
		const mat3& rot3 = frame3.GetJointRotation(i);
		vec3 angles0, angles1, angles2, angles3, angles;
		rot0.ToEulerAnglesZXY(angles0);
		rot1.ToEulerAnglesZXY(angles1);
		rot2.ToEulerAnglesZXY(angles2);
		rot3.ToEulerAnglesZXY(angles3);
		//angles = CubicVec3(angles0, angles1, angles2, angles3, fPerc);
		angles = angles1 * (1.0f - fPerc) + angles2 * (fPerc);
		targetFrame.m_rotationData[i].FromEulerAnglesZXY(angles);
		targetFrame.m_quaternionData[i].FromRotation(targetFrame.m_rotationData[i]);
	}
}
//////////////////////////////////////////////////////////////////////////
// Slerp interpolation from frame[start] to frame[end]
// Inputs are: frame0 = frame[start], frame1 = frame[end], targetFrame holds the result, fPerc is between [0,1] for interpolation
// The root translation is interpolated as 3D vector linear interpolation
// The joint rotations are interpolated using Quaternion::Slerp()
// Remember that every frame carries both rotation matrix and quaternion,
//		In order to keep data consistency, you need to update the Quaternion when you modify rotation matrix and vice versa.
void Frame::Slerp(const Frame& frame0, const Frame& frame1, Frame& targetFrame, float fPerc)
{
	// Replace the following code with your code
	// You need to implement the root translation interpolation

	targetFrame.m_rootTranslation = frame0.m_rootTranslation * (1.0f - fPerc) + frame1.m_rootTranslation * fPerc;
	for (unsigned int i = 0; i < frame0.m_jointCount; i++)
	{
		// Slerp the joint rotations
		// Make sure of the data consistency
		const auto& q0 = frame0.m_quaternionData[i];
		const auto& q1 = frame1.m_quaternionData[i];
		auto q = Quaternion::Slerp(fPerc, q0, q1);

		targetFrame.SetJointRotation(i, q);		
	}
}

//////////////////////////////////////////////////////////////////////////
// Compute the intermediate frame for Squad interpolation on frame[i]
// Inputs are: frame0 = frame[i - 1], frame1 = frame[i], frame2 = frame[i + 1], targetFrame holds the result
// Use Quaternion::Intermediate()
void Frame::IntermediateFrame(const Frame& frame0, const Frame& frame1, const Frame& frame2, Frame& targetFrame)
{
	// Add your code here
	for (unsigned int i = 0; i < frame0.m_jointCount; i++)
	{
		// Slerp the joint rotations
		// Make sure of the data consistency
		const auto& q0 = frame0.m_quaternionData[i];
		const auto& q1 = frame1.m_quaternionData[i];
		const auto& q2 = frame2.m_quaternionData[i];

		auto q = Quaternion::Intermediate(q0, q1, q2);
		targetFrame.SetJointRotation(i, q);
	}
}


//////////////////////////////////////////////////////////////////////////
// Squad interpolation from frame[start] to frame[end]
// Inputs are: frame0 = frame[start], s0  = intermediate frame for frame[start],
//			   s1 = intermediate frame for frame[end], frame1 = frame[end],
//			   targetFrame holds the result, fPerc is between [0,1] for interpolation
// The root translation is interpolated as 3D vector linear interpolation
// The joint rotations are interpolated using Quaternion::Squad()
// Remember that every frame carries both rotation matrix and quaternion,
//		In order to keep data consistency, you need to update the Quaternion when you modify rotation matrix and vice versa.
void Frame::Squad(const Frame& frame0, const Frame& s0,const Frame& s1, const Frame& frame1,
				  Frame& targetFrame, float fPerc)
{
	// Add your code here

	targetFrame.m_rootTranslation = frame0.m_rootTranslation * (1.0f - fPerc) + frame1.m_rootTranslation * fPerc;
	for (unsigned int i = 0; i < frame0.m_jointCount; i++)
	{
		// Slerp the joint rotations
		// Make sure of the data consistency
		const auto& q0 = frame0.m_quaternionData[i];
		const auto& q1 = frame1.m_quaternionData[i];
		const auto& a0 = s0.m_quaternionData[i];
		const auto& a1 = s1.m_quaternionData[i];
		auto q = Quaternion::Squad(fPerc, q0, a0, a1, q1);		

		targetFrame.SetJointRotation(i, q);
	}
}

//////////////////////////////////////////////////////////////////////////
//
// Motion
//
//////////////////////////////////////////////////////////////////////////

Motion::Motion()
{
	m_jointCount = 0;
	m_name = "unknown";
	m_frameCount = 0;
	m_currentFrame = 0;
}

Motion::~Motion()
{
	FreeSpace();
}

bool Motion::LoadFromFile(ifstream& inFile, Skeleton* pSkeleton)
{
	string readString;
	unsigned int frameCount;
	inFile >> readString;
	if (readString != "MOTION")
		return false;
	inFile >> readString;
	if(readString != "Frames:") 
		return false;
	inFile >> frameCount;
	inFile >> readString; // "Frame"
	getline(inFile, readString); // " Time: 0.033333"

	FreeSpace();
	m_jointCount = pSkeleton->GetJointCount();
	m_frameCount = frameCount;
	AllocateSpace();
	
	for (vector<Frame*>::const_iterator iter = m_keyFrames.begin(); iter != m_keyFrames.end(); iter++)
	{
		(*iter)->LoadFromFile(inFile, pSkeleton);
	}
	
	m_currentFrame = 0;
	return true;
}

void Motion::SaveToFile(ofstream& outFile, Skeleton* pSkeleton)
{
	outFile << "MOTION" << endl;
	outFile << "Frames: " << m_frameCount << endl;
	outFile << "Frame Time: 0.033333" << endl;
	for (vector<Frame*>::const_iterator iter = m_keyFrames.begin(); iter != m_keyFrames.end(); iter++)
	{
		(*iter)->SaveToFile(outFile, pSkeleton);
	}
}

void Motion::FreeSpace()
{
	for (vector<Frame*>::const_iterator iter = m_keyFrames.begin(); iter != m_keyFrames.end(); iter++)
	{
		delete (*iter);
	}
	m_keyFrames.clear();
	m_frameCount = 0;
	m_jointCount = 0;
}

void Motion::AllocateSpace()
{
	Frame* pFrameData;
	for (unsigned int i = 0; i < m_frameCount; i++)
	{
		pFrameData = new Frame();
		pFrameData->SetJointCount(m_jointCount);
		m_keyFrames.push_back(pFrameData);
	}
}


void Motion::Clone(const Motion& motionData)
{
	// Free old frames
	if (m_keyFrames.size() > 0)
	{
		FreeSpace();
	}
	
	// Clone info
	m_name = motionData.m_name;
	m_jointCount = motionData.m_jointCount;
	m_frameCount = motionData.m_frameCount;

	//Clone animation data
	Frame* pFrameData;
	for (unsigned int i = 0; i < m_frameCount; i++)
	{
		pFrameData = new Frame();
		pFrameData->Clone(*(motionData.m_keyFrames[i]));
		m_keyFrames.push_back(pFrameData);
	}
}

Frame* Motion::GetFrameAt(const unsigned int& index)
{
	assert(index >= 0 && index < m_frameCount);
	return m_keyFrames[index];
}

Frame* Motion::GetCurrentFrame()
{
	return m_keyFrames[m_currentFrame];
}

Motion* Motion::CreateMotion(unsigned int jointCount, unsigned int frameCount, string name)
{
	Motion* pMotion = new Motion();
	pMotion->m_jointCount = jointCount;
	pMotion->m_frameCount = frameCount;
	pMotion->m_name = name;
	pMotion->AllocateSpace();
	return pMotion;
}

void Motion::Update( int deltaFrame )
{
	int num = int(m_currentFrame) + deltaFrame;
	while (num >= int(m_frameCount))
		num -= m_frameCount;
	while (num < 0)
		num += m_frameCount;
	m_currentFrame = unsigned int(num);
}
//////////////////////////////////////////////////////////////////////////
//
// Player
//
//////////////////////////////////////////////////////////////////////////

Player::Player()
{
	m_currentMotion = &m_motion1;
	m_motionIndex = eMotion1;
    m_IKType = eCCD;
}

Player::~Player()
{

}

bool Player::LoadBVHFile(ifstream& inFile)
{
	if (m_skeleton.LoadFromFile(inFile) == false)
		return false;
	if (m_currentMotion->LoadFromFile(inFile, &m_skeleton) == false)
		return false;
	return true;
}

bool Player::SaveBVHFile(ofstream& outFile)
{
	m_skeleton.SaveToFile(outFile);
	m_currentMotion->SaveToFile(outFile, &m_skeleton);
	return true;
}

void Player::UpdateFrame()
{
	if (IsValid())
	{
		m_skeleton.ReadFromFrame(m_currentMotion->GetCurrentFrame());
	}
}

void Player::SetCurrentMotion(const MotionType& index)
{
	m_motionIndex = index;
	switch(index)
	{
	case eMotion1:
		m_currentMotion = &m_motion1;
		break;
	case eMotion2:
		m_currentMotion = &m_motion2;
	    break;
	case eMotion3:
		m_currentMotion = &m_motion3;
	    break;
	}
}
Motion* Player::GetCurrentMotion()
{
	return m_currentMotion;
}

Skeleton* Player::GetSkeleton()
{
	return &m_skeleton;
}

bool Player::IsValid()
{
	return (m_skeleton.GetJointCount() > 0 && m_currentMotion && m_currentMotion->GetFrameCount() > 0);
}

void Player::SaveFrame()
{
	m_skeleton.WriteToFrame(m_currentMotion->GetCurrentFrame());
}

bool Player::BlendingReady(unsigned int& frameCount0, unsigned int& frameCount1)
{
	frameCount0 = m_motion1.GetFrameCount();
	frameCount1 = m_motion2.GetFrameCount();
	return (m_motion1.GetJointCount() == m_motion2.GetJointCount() && m_motion1.GetFrameCount() > 0 && m_motion2.GetFrameCount() >0);
}

void Player::Blend( const unsigned int& startFrame, const unsigned int& endFrame, const unsigned int& interpFrame, const BlendType& type )
{
	// Compute total frame number for the target motion
	unsigned int totalFrame = startFrame + interpFrame + m_motion2.GetFrameCount() - endFrame + 1;
	m_motion3.FreeSpace();
	m_motion3.m_jointCount = m_skeleton.GetJointCount();
	m_motion3.m_frameCount = totalFrame;
	m_motion3.AllocateSpace();

	// m_motion3[0, ..., startFrame] = m_motion1[0, ..., startFrame]
	for (unsigned int i = 0; i < startFrame + 2; i++)
	{
		m_motion3.m_keyFrames[i]->Clone(*(m_motion1.m_keyFrames[i]));
	}

	// Estimate root translation and rotation for m_motion3[startFrame + interpFrame + 1]
	const vec3& rootPos0 = m_motion1.GetFrameAt(startFrame - 1)->GetRootTranslation();
	const vec3& rootPos1 = m_motion1.GetFrameAt(startFrame)->GetRootTranslation();
	const vec3& rootPos2 = m_motion2.GetFrameAt(endFrame)->GetRootTranslation();
	const vec3& rootPos3 = m_motion2.GetFrameAt(endFrame + 1)->GetRootTranslation();
	const mat3& rootRot1 = m_motion1.GetFrameAt(startFrame)->GetJointRotation(0);
	const mat3& rootRot2 = m_motion2.GetFrameAt(endFrame)->GetJointRotation(0);

	vec2 vel1(rootPos1[VX] - rootPos0[VX], rootPos1[VZ] - rootPos0[VZ]);
	vec2 vel2(rootPos3[VX] - rootPos2[VX], rootPos3[VZ] - rootPos2[VZ]);
	float l1 = vel1.Length();
	float l2 = vel2.Length();
	float dist = (l1 + l2) * interpFrame / 2.0f;
	vel1 /= l1;
	vel2 /= l2;	
	vec3 rootPos = rootPos1 + vec3(vel1[VX] * dist, 0.0f, vel1[VY] * dist);
	rootPos[VY] = rootPos2[VY];
	//if (vel1.Length() < 2.0f || vel2.Length() < 2.0f)
	//{
		vel1 = vec2(rootRot1[VX][VZ], rootRot1[VZ][VZ]);
		vel2 = vec2(rootRot2[VX][VZ], rootRot2[VZ][VZ]);
		vel1.Normalize();
		vel2.Normalize();
	//}	
	float angle1 = atan2(vel1[VX], vel1[VY]);
	float angle2 = atan2(vel2[VX], vel2[VY]);
	float deltaAngle = AngleDiff(angle2, angle1);
	mat3 deltaRot = mat3::Rotation3DRad(axisY, deltaAngle).Transpose();

	// m_motion3[startFrame + interpFrame + 1, ..., totalFrame] = m_motion2[endFrame, ..., frameCount] with proper root translation and rotation
	vec3 r, o = m_motion2.GetFrameAt(endFrame)->GetRootTranslation();
	Frame* pFrame;
	mat3 tmpRot;
	unsigned int i, j;
	for (i = endFrame - 1, j = startFrame + interpFrame; i < m_motion2.GetFrameCount(); i++, j++)
	{
		pFrame = m_motion3.GetFrameAt(j);
		pFrame->Clone(*(m_motion2.m_keyFrames[i]));
		r = pFrame->GetRootTranslation() - o;
		r = deltaRot * r + rootPos;
		pFrame->SetRootTranslation(r);
		pFrame->SetJointRotation(0, deltaRot * pFrame->GetJointRotation(0));
	}

	// m_motion3[startFrame + 1, ..., startFrame + interpFrame] are computed from proper interpolations including
	// root translation interpolation
	// joint rotation interpolation using Cubic, SLERP and SQUAD
	float fPerc;
	Frame s0, s1;
	s0.SetJointCount(m_motion3.m_jointCount);
	s1.SetJointCount(m_motion3.m_jointCount);
	for (i = startFrame + 1, j = 0; i < startFrame + interpFrame + 1; i++, j++)
	{
		fPerc = float(j + 1) / float(interpFrame + 1);
		pFrame = m_motion3.GetFrameAt(i);
		switch(type){
		case eCUBIC: 
			Frame::Cubic(*(m_motion3.m_keyFrames[startFrame - 1]), *(m_motion3.m_keyFrames[startFrame]), 
				  *(m_motion3.m_keyFrames[startFrame + interpFrame + 1]), *(m_motion3.m_keyFrames[startFrame + interpFrame + 2]),
				  *(m_motion3.m_keyFrames[i]), fPerc); 
			break;
		case eSLERP: 
			Frame::Slerp(*(m_motion3.m_keyFrames[startFrame]), *(m_motion3.m_keyFrames[startFrame + interpFrame + 1]),
						 *(m_motion3.m_keyFrames[i]), fPerc); 
			break;
		case eSQUAD:
			Frame::IntermediateFrame(*(m_motion3.m_keyFrames[startFrame - 1]), *(m_motion3.m_keyFrames[startFrame]),
									 *(m_motion3.m_keyFrames[startFrame + 1]), s0);
			Frame::IntermediateFrame(*(m_motion3.m_keyFrames[startFrame + interpFrame]), *(m_motion3.m_keyFrames[startFrame + interpFrame + 1]),
									 *(m_motion3.m_keyFrames[startFrame + interpFrame + 2]), s1);
			Frame::Squad(*(m_motion3.m_keyFrames[startFrame]), s0, 
						 s1, *(m_motion3.m_keyFrames[startFrame + interpFrame + 1]),
						 *(m_motion3.m_keyFrames[i]), fPerc); 
			break;
		}
	}
}

const vec3& Player::GetGoalPosition() const
{
	return m_skeleton.m_goalPosition;
}

void Player::SetGoalPosition(const vec3& pos)
{
	m_skeleton.m_goalPosition = pos;
}

void Player::SolveIK()
{
	switch( m_IKType )
	{
	case eCCD:
		m_skeleton.SolveIKCCD();
		break;

	case eJacobianPinv:
		m_skeleton.SolveIKJacobianPseudoInverse();
		break;

	case eJacobianT:
		m_skeleton.SolveIKJacobianTranspose();
		break;

	default:
		// We should never be here.
		;
	}
}

const Player::IKType& Player::GetIKType() const
{
	return m_IKType;
}
void Player::SetIKType( const IKType& type )
{
	m_IKType = type;
}
