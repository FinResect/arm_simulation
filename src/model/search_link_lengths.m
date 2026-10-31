function search = search_link_lengths(core, ranges, counts, options)
% Shared structural lengths; each target retains independent joints/base height.
% Numerical feasibility and local refinement do not establish a global optimum.
    arguments
        core TechCore
        ranges (6, 2) double {mustBeReal, mustBeFinite}
        counts (1, :) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive}
        options.InitialLengths (1, 2) double {mustBeReal, mustBeFinite, mustBeNonnegative} = [0.14804, 0.20508]
        options.Resolution (1, 1) double {mustBeReal, mustBeFinite, mustBePositive} = 0.001
        options.MaxCandidates (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = 100
        options.SampleSeed (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(options.SampleSeed, 4294967295)} = 0
        options.CheckpointFile (1, 1) string = ""
        options.EvaluationOptions (1, :) cell = {}
    end
    % Replaying this RNG state gives every candidate exactly the same targets.
    previousRng = rng;
    rngCleanup = onCleanup(@() rng(previousRng));
    rng(options.SampleSeed, 'twister');
    sampleState = rng;
    evalOptions = options.EvaluationOptions;
    if mod(numel(evalOptions), 2) ~= 0
        error('LinkSearch:InvalidOptions', 'EvaluationOptions must contain name/value pairs.');
    end
    names = string(evalOptions(1:2:end));
    if numel(unique(lower(names))) ~= numel(names)
        error('LinkSearch:DuplicateOptions', 'EvaluationOptions must not contain duplicate names.');
    end
    % Probe settings match the complete evaluator, including per-sample substreams.
    heightRange = [0.290, 0.490];
    attempts = 3;
    positionTolerance = 1e-4;
    orientationTolerance = 1e-3;
    ikSeed = 0;
    for k = 1:numel(names)
        switch lower(names(k))
            case "baseheightrange", heightRange = evalOptions{2*k};
            case "attempts", attempts = evalOptions{2*k};
            case "positiontolerance", positionTolerance = evalOptions{2*k};
            case "orientationtolerance", orientationTolerance = evalOptions{2*k};
            case "ikseed", ikSeed = evalOptions{2*k};
        end
    end
    % Let sample_range validate sampling arguments before the first IK call.
    sampling = "grid";
    maxSamples = 100000;
    for k = 1:numel(names)
        switch lower(names(k))
            case "sampling", sampling = evalOptions{2*k};
            case "maxsamples", maxSamples = evalOptions{2*k};
        end
    end
    cloud = core.sample_range(ranges, counts, 'Sampling', sampling, 'MaxSamples', maxSamples);
    transforms = pagemtimes(inv(core.T_base_scene), cloud.targetTransforms);
    n = size(cloud.samples, 1);
    probeIndices = unique(round(linspace(1, n, min(8, n))));
    search = struct('foundFeasible', false, 'bestLengths', [NaN, NaN], ...
        'bestTotalLength', Inf, 'bestResult', [], 'history', zeros(0, 5), ...
        'status', 'running', 'resolution', options.Resolution, ...
        'refinementCompleted', false, 'sampleSeed', options.SampleSeed, ...
        'ranges', ranges, 'counts', counts, 'evaluationOptions', {evalOptions}, ...
        'initialLengths', options.InitialLengths, 'maxCandidates', options.MaxCandidates);
    timer = tic;
    tested = zeros(0, 2);
    span = max(sum(options.InitialLengths), options.Resolution);
    step = max(span / 4, options.Resolution);
    queue = options.InitialLengths;
    coarseScanned = false;
    while size(search.history, 1) < options.MaxCandidates
        if isempty(queue)
            if ~search.foundFeasible || ~coarseScanned
                if search.foundFeasible
                    span = search.bestTotalLength;
                    coarseScanned = true;
                else
                    span = span * 2;
                    step = max(span / 4, options.Resolution);
                end
                % Explore multiple ratios, including zero-length boundaries.
                axisValues = unique([0:step:span, span]);
                [d3, d5] = ndgrid(axisValues, axisValues);
                queue = [d3(:), d5(:)];
                queue = queue(sum(queue, 2) <= span + 1e-12, :);
                if search.foundFeasible
                    queue = [queue; neighbours(search.bestLengths, step)];
                end
            else
                % Repeated descent, then halve the step down to the requested scale.
                if step <= options.Resolution
                    search.refinementCompleted = true;
                    search.status = 'local_refinement_completed';
                    break;
                end
                step = max(step / 2, options.Resolution);
                queue = neighbours(search.bestLengths, step);
            end
            [~, order] = sort(sum(queue, 2));
            queue = queue(order, :);
        end
        lengths = queue(1, :);
        queue(1, :) = [];
        if any(lengths < 0) || sum(lengths) >= search.bestTotalLength - 1e-12 || ...
                any(all(abs(tested - lengths) < 1e-12, 2))
            continue;
        end
        tested(end + 1, :) = lengths;
        candidateNumber = size(search.history, 1) + 1;
        fprintf('\nLink search %d/%d: d3 %.3f mm, d5 %.3f mm, total %.3f mm.\n', ...
            candidateNumber, options.MaxCandidates, lengths * 1000, sum(lengths) * 1000);
        arm = ArmModel('Show', false, 'LinkLengths', lengths);
        rejected = false;
        for index = probeIndices(:).'
            fprintf('Probe sample %d: IK started, search elapsed %.1f s.\n', index, toc(timer));
            stream = RandStream('Threefry', 'Seed', ikSeed);
            stream.Substream = index;
            [~, info] = arm.inverse_lift_transform(transforms(:, :, index), heightRange, ...
                'Attempts', attempts, 'PositionTolerance', positionTolerance, ...
                'OrientationTolerance', orientationTolerance, 'RandomStream', stream);
            if ~info.success
                probeIndices = unique([index, probeIndices], 'stable');
                probeIndices = probeIndices(1:min(8, numel(probeIndices)));
                rejected = true;
                fprintf('Rejected by probe sample %d (numerical IK failure).\n', index);
                break;
            end
        end
        passed = NaN; % A rejected probe does not measure the full success count.
        feasible = false;
        if ~rejected
            rng(sampleState);
            result = core.evaluate(arm, ranges, counts, evalOptions{:});
            passed = nnz(result.success);
            feasible = result.allSampledPosesPassed;
            failed = find(~result.success, 8, 'first').';
            probeIndices = unique([failed, probeIndices], 'stable');
            probeIndices = probeIndices(1:min(8, numel(probeIndices)));
            if feasible
                search.foundFeasible = true;
                search.bestLengths = lengths;
                search.bestTotalLength = sum(lengths);
                search.bestResult = result;
                fprintf('New best: %.3f + %.3f = %.3f mm; all %d poses passed.\n', ...
                    lengths * 1000, sum(lengths) * 1000, n);
                if coarseScanned
                    queue = [neighbours(lengths, step); queue];
                end
            end
        end
        search.history(end + 1, :) = [lengths, passed, double(feasible), toc(timer)];
        search.elapsedTime = toc(timer);
        if strlength(options.CheckpointFile) > 0
            save(options.CheckpointFile, 'search');
        end
        if search.bestTotalLength <= 1e-12
            search.refinementCompleted = true;
            search.status = 'zero_total_length';
            break;
        end
    end
    if strcmp(search.status, 'running')
        search.status = 'candidate_budget_exhausted';
    end
    search.elapsedTime = toc(timer);
    if strlength(options.CheckpointFile) > 0
        save(options.CheckpointFile, 'search');
    end
    fprintf('Link search stopped: %s; %d candidates, %.1f s.\n', ...
        search.status, size(search.history, 1), search.elapsedTime);
    if ~search.foundFeasible
        fprintf('No all-sample feasible lengths found; this does not prove infeasibility.\n');
    end
end

function candidates = neighbours(lengths, step)
    [a, b] = ndgrid(-1:1, -1:1);
    candidates = lengths + step * [a(:), b(:)];
    candidates = candidates(all(candidates >= -1e-12, 2), :);
    candidates = max(candidates, 0);
end
