#include <KOMO/manipTools.h>
#include <Algo/spline.h>
#include <KOMO/pathTools.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>
#include <Kin/simulation.h>
#include <Optim/NLP_Sampler.h>

#include <chrono>

StringA robotFrameNames, envFrameNames, robotJointNames, envJointNames;
DofL allDofs, robotDofs, envDofs;
double tau = .01;


void loadEnv(rai::Configuration &C){
    auto syncDof = [&](const rai::String path) -> rai::Frame*{
         auto frame = C.addFile(path);
         C.ensure_q();
         return frame;
    };

    syncDof(rai::raiPath("rai-robotModels/scenarios/panda_ranger.g"))
          ->setPosition({0., 2., 0})
          .setQuaternion({1., 0., 0., M_PI_2});
    robotDofs = C.activeDofs;
    robotJointNames = C.getJointNames();
    cout << robotJointNames << endl;
    robotFrameNames = C.getFrameNames();
    // cout << " robot joint names \n " << robotJointNames << endl;
    size_t robotDofCount = robotDofs.N;
    size_t robotJointCount = robotJointNames.N;
    size_t robotFrameCount = robotFrameNames.N;

    syncDof(rai::raiPath("rai-robotModels/kitchen1/kitchen1.g"));
    allDofs = C.activeDofs;
    envDofs = C.activeDofs.sub(robotDofCount, -1);
    envFrameNames = C.getFrameNames().sub(robotFrameCount, -1);
    envJointNames = C.getJointNames().sub(robotJointCount, -1);
    for(auto envFrame : envFrameNames){
        auto f = C.getFrame(envFrame);
        if(f->shape && f->shape->_mesh && f->shape->_mesh->V.N) 
           f->setContact(-1);
    }
    C.view(true);
    size_t dofs = envDofs.N;
    // C.animate();
}

StringA getCollisionPairs(rai::Configuration &C){
    StringA colls;
    for(auto roboFrame : robotFrameNames){
        for(auto envFrame : envFrameNames){
            if(C.getFrame(envFrame)->shape){
                colls.append({roboFrame, envFrame});
            }
        }
    }
    return colls;
}

StringA getCollisionPairsDirect(rai::Configuration &C){
    FrameL collFrames = C.getCollidablePairs();
    collFrames.reshapeFlat();
    StringA colls;
    for(uint i=0; i<collFrames.d0; i++){
        colls.append(collFrames(i)->name);
        // colls.append(collFrames(1)->name);
    }
    return colls;
}

//==================================================================================================//
//
void randomPOA(rai::Configuration &C){
    
}

void randomContactPoint(rai::Configuration &C){

}


//==================================================================================================//

void randomArticulation(){
    
    rai::Configuration C;
    loadEnv(C);
    StringA colls = getCollisionPairsDirect(C);
    // cout << colls << endl;
    C.getFrame("l_panda_base")->getMeshPoints();
    rai::Simulation sim(C, rai::Simulation::_physx, 1);

    for(int i=1;;i++){
        cout << "================= " << i << " =================" <<endl; 
        // random sample an object
        auto randDof = envDofs.rndElem();
        randDof->joint();
        auto joint = randDof->joint();
        auto jointAxis = joint->axis;
        auto jointPosition = randDof->frame->getPosition();
        auto obj = randDof->frame;
        FrameL objSubTree = {obj};
        obj->getSubtree(objSubTree);
        for(uint i=0; i<objSubTree.N;i++){
            if(!objSubTree(i)->shape) objSubTree.remove(i);
        }
        rai::Frame *randFrame = objSubTree.rndElem();

        //random sample the target state
        cout << "######## \n" << randDof->limits.reshape(2,-1).nd<< endl;
        arr target = rand(randDof->limits[0], randDof->limits[1]);

        // C.getFrame(randFrame->name)->setColor({1., 0., 0});
        cout << "joint: " << randDof->frame->name << " Limits: " << randDof->limits << endl 
             << "shape: " << randFrame->name << endl
             << "shape type: " << rai::ShapeType(randFrame->getShapeType()) << endl;
             
        // C.view(true);
        // std::cout << "random object" << randFrame->name << std::endl;
        
        ManipulationModelling M(STRING("random Kitchen"));
        M.setup_sequence(C, 2, 1e-2, 1e-1, false, true, true);
        M.freeze_joint({1}, envJointNames);
        StringA envJointNames_delObjJoint;
        envJointNames_delObjJoint = envJointNames;
        envJointNames_delObjJoint.removeValue(randDof->frame->name);
        M.freeze_joint({2}, envJointNames_delObjJoint);
        // The first phase -> approach and grasp
        rai::ShapeType shapeType = randFrame->getShapeType();
        if (shapeType == rai::ST_box) {
            M.grasp_box(1, "l_gripper", randFrame->name, "l_palm");
        } else if (shapeType == rai::ST_capsule) {
            M.grasp_cylinder(1, "l_gripper", randFrame->name, "l_palm");
        } else if (shapeType == rai::ST_cylinder) {
            M.grasp_cylinder(1, "l_gripper", randFrame->name, "l_palm");
        } else {
            continue;
        }
        auto shapeSize = randFrame->getSize(); 
        auto pose = randFrame->ensure_X();
        C.addFrame("visualize")->setColor({1., 0., 0., 0.5})
                                .setShape(shapeType, 1.5*shapeSize)
                                .setPose(pose);
        cout << "Visualize Shape Type " << shapeType << endl;
        C.view(true, "Random Frame Visualization");
        C.delFrame("visualize"); 
        C.view();  
        ///Here is a bug, if do not view, the frame shape are not updated!!!  
        ///The configuration will only updated when view() is called and updateConfiguration() is executed??
        // continue;

        // The second sequence
        M.komo->addObjective({2}, FS_qItself, {joint->frame->name}, OT_eq, {1e1}, target);
        M.komo->addFrameDof("grasp_frame", "l_gripper", rai::JointType::JT_free, true, randFrame->name);
        M.komo->addObjective({1, 2}, FS_poseDiff, {"grasp_frame", randFrame->name}, OT_eq, {1e1});
        
        auto MStart = std::chrono::steady_clock::now();
        M.solve(0);
        auto MEnd = std::chrono::steady_clock::now();
        std::chrono::duration<double> MDuration = MEnd - MStart;
        std::cout << "Task M solving time: " << MDuration.count() << std::endl;
        if(!M.ret->feasible) continue;
        // M.play(C);
        
        //Use RRT to find the path
        std::shared_ptr<rai::RRT_PathFinder> R1 = M.sub_rrt(0, {}, robotJointNames);
        R1->opt.set_stepsize(.05);
        R1->solve();

        if(!R1->ret->feasible) continue;

        auto M1 = M.sub_motion(0, true, 1e-3, 1e-2, true, true, true, robotJointNames);
        M1->komo->initWithPath_qOrg(R1->get_resampledPath(M1->komo->T));
        M1->approach({0.8, 1.}, "l_gripper");
        // M1->no_collisions({.2, .8}, colls);
        M1->freeze_joint({1.}, envJointNames);
        auto M1Start = std::chrono::steady_clock::now();
        M1->solve();
        auto M1End = std::chrono::steady_clock::now();
        std::chrono::duration<double> M1Duration = M1End - M1Start;
        std::cout << "Subtask M1 solving time: " << M1Duration.count() << std::endl;
        if(!M1->ret->feasible) 
        {
            continue;
        }
        auto path1 = M1->path;

        rai::Configuration C2 = C;
        C2.ensure_q();
        auto allJointNames = C2.getJointNames();
        C2.selectJointsByName(robotJointNames);
        M1->play(C2, path1.d0*0.2);
        // for(int i = 0; i < path1.d0; i++){
        //     C2.setJointState(path1[i]);
        //     C2.view();
        //     rai::wait(0.2);
        // }

        auto M2Joints = robotJointNames;
        M2Joints.append({randDof->frame->name});
        auto M2 = M.sub_motion(1, true, 1e-2, 1e-1, true, true, true, M2Joints);
        M2->freeze_joint({1.}, envJointNames_delObjJoint);
        // M2->komo->addObjective({0., 1.}, FS_poseDiff, {"grasp_frame", randFrame->name}, OT_eq, {1e1});
        M2->solve();
        if(!M2->ret->feasible)
        {
            continue;
        }
        auto path2 = M2->path;

        C2.selectJointsByName(allJointNames);
        C2.selectJointsByName(M2Joints);
        M2->play(C2, path2.d0*0.2);
        // for(int i = 0; i < path2.d0; i++){
        //     C2.setJointState(path2[i]);
        //     C2.view();
        //     rai::wait(0.2);
        // }

        C.view(true);
        

//Simulation
        //Phase1
        // sim.C.selectJointsByName(robotJointNames);
        // sim.resetSplineRef();
        // sim.setSplineRef(path1, {1.});
        // while(sim.getTimeToSplineEnd()>.0){
        //     sim.step({}, tau, rai::Simulation::_spline);
        //     C.view();
        //     rai::wait(tau);
        // }

        // sim.moveGripper("l_gripper", 0.);
        // while(!sim.gripperIsDone("l_gripper")){
        //     sim.step({}, tau, rai::Simulation::_spline);
        //     C.view();
        //     rai::wait(tau);
        // }

        // // sim.C.selectJoints(allDofs);
        // // sim.C.selectJointsByName(robotJointNames);
        // sim.setSplineRef(path2.cols(0,-1), {3.});
        // while(sim.getTimeToSplineEnd()>.0){
        //     sim.step({}, tau, rai::Simulation::_spline);
        //     C.view();
        //     rai::wait(tau);
        // }
        
        // sim.moveGripper("l_gripper", .08);
        // while(!sim.gripperIsDone("l_gripper")){
        //     sim.step({}, tau, rai::Simulation::_spline);
        //     rai::wait(tau);
        // }
        // sim.C.selectJoints(allDofs);
    }

}


int main(int argc, char** argv){
    rai::initCmdLine(argc, argv);
    rai::setRaiPath("/home/shiping/pyvenv/lib/python3.12/site-packages/robotic");
    std::cout << "rai path: " << rai::raiPath() << std::endl;

    randomArticulation();

    return 0;
}