% Auto-generated.  Do not edit!

% [reqmsg,resmsg] = core_testsrv1()
%
function [reqmsg,resmsg] = comp_ehow_EhowToOWL()
if( nargout > 0 )
    reqmsg = comp_ehow_Request();
end
if( nargout > 0 )
    resmsg = comp_ehow_Response();
end

% Auto-generated.  Do not edit!

% msg = comp_ehow_Request()
%
% Request message type, fields include:
% string command

% //! \htmlinclude Request.msg.html
function msg = comp_ehow_Request()

msg = [];
msg.create_response_ = @comp_ehow_Response;
msg.command = '';
msg.md5sum_ = @comp_ehow_Request___md5sum;
msg.server_md5sum_ = @comp_ehow_Request___server_md5sum;
msg.server_type_ = @comp_ehow_Request___server_type;
msg.type_ = @comp_ehow_Request___type;
msg.serializationLength_ = @comp_ehow_Request___serializationLength;
msg.serialize_ = @comp_ehow_Request___serialize;
msg.deserialize_ = @comp_ehow_Request___deserialize;
msg.message_definition_ = @comp_ehow_Request___message_definition;

function x = comp_ehow_Request___md5sum()
x = '';

function x = comp_ehow_Request___server_md5sum()
x = '757be2a3a997cc31d4d3125810ee07f5';

function x = comp_ehow_Request___server_type()
x = '';

function x = comp_ehow_Request___message_definition()
x = [    '\n' ...
];

function x = comp_ehow_Request___type()
x = 'comp_ehow/EhowToOWLRequest';

function l__ = comp_ehow_Request___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.command);

function dat__ = comp_ehow_Request___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.command), 'uint32');
fwrite(fid__, msg__.command, 'uint8');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = comp_ehow_Request___deserialize(dat__, fid__)
msg__ = comp_ehow_Request();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.command = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = comp_ehow_Request___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

% msg = comp_ehow_Response()
%
% Response message type, fields include:
% string owl_instructions

% //! \htmlinclude Response.msg.html
function msg = comp_ehow_Response()

msg = [];
msg.owl_instructions = '';
msg.md5sum_ = @comp_ehow_Response___md5sum;
msg.server_md5sum_ = @comp_ehow_Response___server_md5sum;
msg.server_type_ = @comp_ehow_Response___server_type;
msg.type_ = @comp_ehow_Response___type;
msg.serializationLength_ = @comp_ehow_Response___serializationLength;
msg.serialize_ = @comp_ehow_Response___serialize;
msg.deserialize_ = @comp_ehow_Response___deserialize;
msg.message_definition_ = @comp_ehow_Response___message_definition;

function x = comp_ehow_Response___md5sum()
x = '';

function x = comp_ehow_Response___server_md5sum()
x = '757be2a3a997cc31d4d3125810ee07f5';

function x = comp_ehow_Response___server_type()
x = '';

function x = comp_ehow_Response___message_definition()
x = [    '\n' ...
];

function x = comp_ehow_Response___type()
x = 'comp_ehow/EhowToOWLResponse';

function l__ = comp_ehow_Response___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.owl_instructions);

function dat__ = comp_ehow_Response___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.owl_instructions), 'uint32');
fwrite(fid__, msg__.owl_instructions, 'uint8');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = comp_ehow_Response___deserialize(dat__, fid__)
msg__ = comp_ehow_Response();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.owl_instructions = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = comp_ehow_Response___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

