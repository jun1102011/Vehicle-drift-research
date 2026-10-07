function metadata = save_numeric_archive(out, target)
% 将完整 SimulationOutput 中的数值日志保存为单份 MAT v7 归档。
% 顶层变量均为原生 double 数组；archive_metadata_json 为 UTF-8 uint8。
% 禁止覆盖已有文件；遇到不能表示的日志立即报错，避免静默丢失。
assert(isa(out, 'Simulink.SimulationOutput'), '输入必须是 SimulationOutput');
assert(~isfile(target), '归档已存在，禁止覆盖: %s', target);
names = out.who;
records = struct;
metadata = struct('archive_version', 1, 'format', 'MAT-v7-flat-numeric', ...
    'source_class', 'Simulink.SimulationOutput', 'source_fields', {names}, ...
    'error_message', char(out.ErrorMessage), 'channels', struct, 'logsout_count', 0);
sm = out.SimulationMetadata;
metadata.simulation_metadata = struct;
for k = 1:numel(properties(sm))
    props = properties(sm);
    key = props{k};
    value = sm.(key);
    encoded = jsonencode(value); % 无法序列化就中止，不丢失运行元数据。
    assert(numel(encoded) < 1000000, 'SimulationMetadata 字段异常过大: %s', key);
    metadata.simulation_metadata.(key) = value;
end
for k = 1:numel(names)
    name = names{k};
    value = out.get(name);
    if strcmp(name, 'logsout')
        assert(isa(value, 'Simulink.SimulationData.Dataset'), 'logsout 类型未知');
        metadata.logsout_count = value.numElements;
        for j = 1:value.numElements
            element = value.getElement(j);
            key = sprintf('logsout_%03d', j);
            assert(~isfield(records, key), '归档键冲突');
            [records.(key), info] = normalize_log(element.Values);
            info.source_field = 'logsout';
            info.element_index = j;
            info.element_name = char(element.Name);
            info.block_path = char(element.BlockPath.getBlock(1));
            metadata.channels.(key) = info;
        end
    elseif strcmp(name, 'tout')
        assert(isnumeric(value) && isreal(value) && isa(value, 'double'), 'tout 非实数 double');
        records.tout = value(:);
        metadata.channels.tout = describe_channel(records.tout, 'double', 'time_vector', []);
        metadata.channels.tout.source_field = 'tout';
    else
        assert(~isfield(records, name), '归档键冲突');
        [records.(name), info] = normalize_log(value);
        info.source_field = name;
        metadata.channels.(name) = info;
    end
end
assert(numel(fieldnames(records)) == numel(names) - any(strcmp(names, 'logsout')) + metadata.logsout_count, ...
    '日志数量不符，禁止保存不完整归档');
records.archive_metadata_json = unicode2native(jsonencode(metadata), 'UTF-8');
save(target, '-struct', 'records', '-v7');
end

function [z, info] = normalize_log(value)
% 按旧 asarray 约定：第一列为时间，其余列为原生样本值。
if isa(value, 'timeseries')
    t = value.Time;
    data = value.Data;
    if value.IsTimeFirst
        a = reshape(data, numel(t), []);
    else
        a = reshape(data, [], numel(t))';
    end
    source_class = 'timeseries';
    layout = 'time_then_flattened_data';
    identity = struct('name', char(value.Name), ...
        'data_units', char(value.DataInfo.Units), ...
        'time_units', char(value.TimeInfo.Units), ...
        'is_time_first', logical(value.IsTimeFirst));
elseif isstruct(value) && isscalar(value) && isfield(value, 'time') && isfield(value, 'signals')
    t = value.time;
    data = value.signals.values;
    if size(data, 1) == numel(t)
        a = reshape(data, numel(t), []);
    else
        a = reshape(data, [], numel(t))';
    end
    source_class = 'struct_with_time_signals';
    layout = 'time_then_flattened_values';
    identity = struct;
    signal_fields = fieldnames(value.signals);
    for k = 1:numel(signal_fields)
        key = signal_fields{k};
        if ~strcmp(key, 'values')
            jsonencode(value.signals.(key)); % 不可序列化则立即失败。
            identity.(key) = value.signals.(key);
        end
    end
    if isfield(value, 'blockName')
        identity.block_name = value.blockName;
    end
else
    error('不可数值化的日志类型: %s', class(value));
end
assert(isa(t, 'double') && isnumeric(a) && isreal(t) && isreal(a), ...
    '日志时间须为 double，样本值须为实数数值');
original_class = class(a);
if ~isa(a, 'double')
    assert(isa(a, 'int8'), '非 double 日志类型需要单独确认');
    a = double(a); % int8 到 double 精确可逆，元数据保留原类别。
end
t = t(:);
assert(size(a, 1) == numel(t), '日志样本数与时间长度不符');
z = [t, a];
info = describe_channel(z, source_class, layout, size(data));
info.original_data_class = original_class;
info.identity = identity;
end

function info = describe_channel(z, source_class, layout, original_size)
info = struct('class', 'double', 'size', size(z), 'source_class', source_class, ...
    'layout', layout, 'original_data_size', original_size, ...
    'sample_count', size(z, 1));
if ~isempty(z)
    t = z(:, 1);
    info.time_start_s = t(1);
    info.time_end_s = t(end);
    if numel(t) > 1
        dt = diff(t);
        info.uniform_time = all(abs(dt - dt(1)) <= 1e-9 * max(1, abs(dt(1))));
        if info.uniform_time
            info.sample_interval_s = dt(1);
        end
    else
        info.uniform_time = true;
    end
end
end
