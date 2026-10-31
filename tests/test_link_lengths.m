function test_link_lengths()
% Geometry/argument checks only: no IK, optimization, plots or parallel pool.
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'src', 'model'));
    arm = ArmModel('Show', false);
    assert(isequal(arm.get_link_lengths(), [0.14804, 0.20508]));
    original = arm.get_joint_positions();
    longer3 = ArmModel('Show', false, 'LinkLengths', [0.24804, 0.20508]);
    joints3 = longer3.get_joint_positions();
    assert(norm(joints3(1:3, :) - original(1:3, :), 'fro') < 1e-12);
    assert(abs(norm(joints3(4, :) - original(4, :)) - 0.1) < 1e-12);
    longer5 = ArmModel('Show', false, 'LinkLengths', [0.14804, 0.25508]);
    joints5 = longer5.get_joint_positions();
    assert(norm(joints5(1:5, :) - original(1:5, :), 'fro') < 1e-12);
    assert(abs(norm(joints5(6, :) - original(6, :)) - 0.05) < 1e-12);
    assert(isequal(arm.get_joint_positions(), original));
    zero = ArmModel('Show', false, 'LinkLengths', [0, 0]);
    transform = zero.forward_transform(zero.get_current_configuration());
    assert(all(isfinite(transform(:))));
    must_fail(@() ArmModel('Show', false, 'LinkLengths', [-1, 0]));
    must_fail(@() ArmModel('Show', false, 'LinkLengths', [NaN, 0]));
    core = TechCore();
    state = rng;
    must_fail(@() search_link_lengths(core, zeros(6, 2), ones(1, 6), ...
        'EvaluationOptions', {'Attempts', 3, 'Attempts', 4}));
    assert(isequal(rng, state));
    must_fail(@() search_link_lengths(core, zeros(6, 2), ones(1, 6), ...
        'Resolution', 0));
    fprintf('Link-length geometry and argument checks passed (no IK run).\n');
end

function must_fail(callback)
    failed = false;
    try
        callback();
    catch
        failed = true;
    end
    assert(failed, 'Expected input validation to fail.');
end
