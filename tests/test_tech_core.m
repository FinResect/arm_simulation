function test_tech_core()
% Run from the project root: addpath('tests'); test_tech_core
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'src', 'model'), fullfile(root, 'src', 'planning'));
    core = TechCore();
    [tool, cylinder] = core.pose([0.2, 0.3, 0.4], 0, 90, 0);
    assert(norm(cylinder - [eye(3), [0.2; 0.3; 0.4]; 0, 0, 0, 1], 'fro') < 1e-12);
    assert(norm(tool(1:3, 4) - [-0.025; -0.3; 0.4]) < 1e-12);
    for angles = [0, 0, 0; 90, 90, 45; -90, 45, -45; 30, 60, 12].'
        [~, cylinder] = core.pose([0, 0, 0], angles(1), angles(2), angles(3));
        normal = [sind(angles(2)) * cosd(angles(1)); ...
                  sind(angles(2)) * sind(angles(1)); cosd(angles(2))];
        assert(norm(cylinder(1:3, 1) - normal) < 1e-12);
        assert(norm(cylinder(1:3, 1:3).' * cylinder(1:3, 1:3) - eye(3), 'fro') < 1e-12);
    end
    a = core.pose([0, 0, 0], 30, 45, 0);
    b = core.pose([0, 0, 0], 30, 45, 45);
    assert(norm(a(1:3, 4) - b(1:3, 4)) < 1e-12);
    assert(norm(a(1:3, 1:3) - b(1:3, 1:3), 'fro') > 0.1);
    rotation = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    core.set_tool_rotation(rotation);
    b = core.pose([0, 0, 0], 30, 45, 0);
    assert(norm(a(1:3, 4) - b(1:3, 4)) < 1e-12);
    assert(norm(b(1:3, 1:3) - a(1:3, 1:3) * rotation, 'fro') < 1e-12);
    must_fail(@() core.set_tool_rotation(diag([1, 1, -1])));
    ranges = [0, 0.1; 0, 0; 0, 0; -90, 90; 0, 90; -45, 45];
    cloud = core.sample_range(ranges, [2, 5, 5, 3, 2, 2]);
    assert(isequal(size(cloud.samples), [24, 6]));
    assert(isequal(min(cloud.samples, [], 1), ranges(:, 1).'));
    assert(isequal(max(cloud.samples, [], 1), ranges(:, 2).'));
    must_fail(@() core.sample_range(ranges, [2, 1, 1, 3, 2, 2], 'MaxSamples', 23));
    must_fail(@() core.sample_range(ranges, ones(1, 6)));

    rng(17);
    randomCloud = core.sample_range(ranges, [500, 4], 'Sampling', 'random');
    assert(isequal(size(randomCloud.samples), [2000, 6]));
    assert(all(all(randomCloud.samples >= ranges(:, 1).')));
    assert(all(all(randomCloud.samples <= ranges(:, 2).')));
    positions = randomCloud.samples(1:4:end, 1:3);
    assert(size(unique(positions, 'rows'), 1) == 500);
    assert(isequal(randomCloud.samples(:, 1:3), repelem(positions, 4, 1)));
    assert(abs(mean(positions(:, 1)) - 0.05) < 0.005);
    assert(abs(mean(cosd(randomCloud.samples(:, 5))) - 0.5) < 0.03);
    rng(17);
    repeated = core.sample_range(ranges, [500, 4], 'Sampling', 'random');
    assert(isequal(repeated.samples, randomCloud.samples));
    must_fail(@() core.sample_range(ranges, [500, 4], 'Sampling', 'random', 'MaxSamples', 1999));
    must_fail(@() core.sample_range(ranges, ones(1, 6), 'Sampling', 'random'));

    arm = ArmModel('Show', false);
    assert(arm.get_joint_count() == 7);
    q0 = arm.get_current_configuration();
    target = arm.forward_transform(q0);
    joints = arm.get_joint_positions();
    assert(isequal(size(joints), [8, 3]));
    assert(norm(joints(1, :)) < 1e-12);
    assert(norm(joints(end, :) - target(1:3, 4).') < 1e-12);
    [q, info] = arm.inverse_transform(target, 'Attempts', 1);
    assert(info.success && numel(q) == 7);
    assert(norm(arm.forward_transform(q) - target, 'fro') < 0.002);
    pose6 = [target(1:3, 4).', tr2rpy(target)];
    assert(numel(arm.inverse_position(pose6)) == 7);
    must_fail(@() arm.angle_controller(zeros(1, 6), 'rad'));
    must_fail(@() arm.angle_controller([nan, zeros(1, 6)], 'rad'));
    assert(isequal(size(arm.get_end_effector_path([q0; q0])), [2, 3]));
    arm.execute_path([q0; q0], 'Show', false, 'Pause', 0);
    planner = RRT(arm, 'MaxIterations', 1, 'GoalBias', 1);
    path = planner.plan(q0);
    assert(path.success && size(path.jointPath, 2) == 7);

    % Construct a cylinder/tool relationship whose target is a known FK pose.
    [~, ~, baseCylinder] = core.pose([0, 0, 0], 20, 40, 15);
    core.set_tool_rotation(baseCylinder(1:3, 1:3).' * target(1:3, 1:3));
    centreBase = target(1:3, 4) - baseCylinder(1:3, 1:3) * core.p_cylinder_tool;
    centreScene = core.T_base_scene \ [centreBase; 1];
    fixed = [centreScene(1:3); 20; 40; 15];
    ranges = [fixed, fixed];
    ranges(1, 2) = ranges(1, 1) + 10;
    result = core.evaluate(arm, ranges, [2, 1, 1, 1, 1, 1], 'Attempts', 1, 'BaseHeightRange', [0, 0]);
    assert(isequal(result.success, [true; false]));
    assert(isequal(result.successFraction, [1; 0]));
    assert(all(isnan(result.jointConfigurations(2, :))));
    assert(~result.allSampledPosesPassed);
    assert(isequal(arm.get_current_configuration(), q0));
    randomResult = core.evaluate(arm, [fixed, fixed], [1, 2], ...
        'Sampling', 'random', 'Attempts', 1, 'BaseHeightRange', [0, 0]);
    assert(all(randomResult.success));
    assert(isequal(randomResult.positionIndex, [1; 1]));
    assert(isequal(randomResult.successFraction, 1));
    assert(isequal(randomResult.samples, repmat(fixed.', 2, 1)));
    fig = figure('Visible', 'off');
    cleanup = onCleanup(@() close(fig));
    ax = core.draw_cloud(result, axes(fig));
    core.draw_reference(arm, ax);
    assert(isgraphics(ax, 'axes'));
    assert(isscalar(findobj(ax, 'Tag', 'ArmSkeleton')));
    assert(isscalar(findobj(ax, 'Tag', 'baselink (reference height)')));
    assert(isscalar(findobj(ax, 'Tag', 'Scene O''')));
    assert(ax.XLim(1) < 0 && ax.XLim(2) > 0.1);
    assert(ax.ZLim(1) < 0);
    fprintf('TechCore geometry, grid/random sampling, seven-axis IK, RRT and reference plotting checks passed.\n');
end

function must_fail(action)
    caught = false;
    try
        action();
    catch
        caught = true;
    end
    assert(caught, 'Expected invalid input to raise an error.');
end
