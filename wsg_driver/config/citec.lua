 
-- Leroy Ruegemer
-- use advanced positioning mc.move() for control
-- switches to idle mode when foce > FORCE_LIMIT + FORCE_DISABLE_TRESH

-- settings
DEBUG_ENABLE = false
DEFAULT_FORCE_LIMIT = 30
FORCE_DISABLE_TRESH = -10 -- add to limit in checkforce()
SLEEP_TIME = 200

-- command ids
CMD_MEASURE = 0xB0
CMD_POSITION = 0xB1
CMD_IDLE = 0xB2


cmd.register(CMD_MEASURE); -- Measure only
cmd.register(CMD_POSITION); -- Position control
cmd.register(CMD_IDLE); -- Idle
mc.force(DEFAULT_FORCE_LIMIT)

mc.homing(false);

-- Get number of FMF fingers
nfin = 0;
for i = 0,1 do
    if finger.type(i) == "fmf" then
        nfin = nfin + 1;
    else
        break;
    end
end
printf("#FMF fingers: %d\n", nfin)

function hasbit(x, p)
  return x % (p + p) >= p       
end

function dprintf(...)
    if(DEBUG_ENABLE) then
        printf(unpack(arg))
    end
end

function checkforce()
    cur_force, limit = mc.force()
    limit = limit + FORCE_DISABLE_TRESH
    --dprintf("%d, limit:%d\n" , cur_force, limit)
    if cur_force > limit then
        dprintf("over force\n");
        mc.stop();
    end
end
    
function measure()
    
    -- ==== Get measurements ====
    state = gripper.state();
    busy = mc.busy();
    blocked = mc.blocked();
    pos = mc.position();
    speed = mc.speed();
    force = mc.aforce();

    force_l = math.nan; force_r = math.nan;
    if nfin >= 1 then force_l = finger.data(0) end
    if nfin >= 2 then force_r = finger.data(1) end
    --dprintf("finger: %d;%d\n", force_l, force_r);
    if cmd.online() then
        -- Only the lowest byte of state is sent!
        cmd.send(CMD_MEASURE, etob(E_SUCCESS), state % 256,
            { ntob(pos), ntob(speed), ntob(force), ntob(force_l), ntob(force_r)});
    end
end

function process()
    --print("process");
    id, payload = cmd.read();
    dprintf( "Data packet received: ID=%d, payload length=%d\n", id, #payload );

    -- ==== Measurements (1) ====
    busy = mc.busy()
    blocked = mc.blocked()
    pos = mc.position();
    
    -- Position control
    if id == CMD_POSITION then
        dprintf("set_pos\n");
        cmd_width = bton({payload[1],payload[2],payload[3],payload[4]});
        cmd_speed = bton({payload[5],payload[6],payload[7],payload[8]});
        dprintf("Got command %f, %f\n", cmd_width, cmd_speed);
        if busy then mc.stop(); end
        mc.move(cmd_width, math.abs(cmd_speed), 0)
    -- Idle control
    elseif id == CMD_IDLE then
        dprintf("set idle");
        mc.stop();
    end    
    
    
end

while true do
    if cmd.online() then
        pcall(measure);
        pcall(checkforce);
        
        -- process()
        if cmd.available() > 0 then
            if not pcall(process) then
                print("Error occured")
            end
        end
        sleep(SLEEP_TIME)
    else
        -- offline
        sleep(SLEEP_TIME)
    end
        
end
