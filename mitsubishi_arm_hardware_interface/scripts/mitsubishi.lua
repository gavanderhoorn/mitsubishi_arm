-- A Wireshark dissector for the Mitsubishi MXT Real-time robot control protocol
-- Author: Addisu Z. Taddese
--
mitsubishi_mxt_proto = Proto("mitsubishi_mxt","Mitsubishi Real-time Control")

local commands = {[0] = "NULL", [1] = "MOVE", [255] = "END"}
local types = {
  [0] = "NULL", 
  [1] = "POSE",
  [2] = "JOINT",
  [3] = "PULSE"}

local f = mitsubishi_mxt_proto.fields
f.Command = ProtoField.uint16("mitsubishi_mxt.Command","Command", nil, commands)
f.SendType = ProtoField.uint16("mitsubishi_mxt.SendType","SendType", nil, types)
f.RecvType = ProtoField.uint16("mitsubishi_mxt.RecvType","RecvType", nil, types)
f.reserve = ProtoField.uint16("mitsubishi_mxt.reserve","reserve")
f.SendIOType = ProtoField.uint16("mitsubishi_mxt.SendIOType","SendIOType", nil, types)
f.RecvIOType = ProtoField.uint16("mitsubishi_mxt.RecvIOType","RecvIOType", nil, types)
f.BitTop = ProtoField.uint16("mitsubishi_mxt.BitTop","BitTop")
f.BitMask = ProtoField.uint16("mitsubishi_mxt.BitMask","BitMask")
f.IoData = ProtoField.uint16("mitsubishi_mxt.IoData","IoData")
f.TCount = ProtoField.uint16("mitsubishi_mxt.TCount","TCount")
f.CCount = ProtoField.uint16("mitsubishi_mxt.CCount","CCount")
f.RecvType1 = ProtoField.uint16("mitsubishi_mxt.RecvType1","RecvType1", nil, types)
f.reserve1 = ProtoField.uint16("mitsubishi_mxt.reserve1","reserve1")
f.RecvType2 = ProtoField.uint16("mitsubishi_mxt.RecvType2","RecvType2", nil, types)
f.reserve2 = ProtoField.uint16("mitsubishi_mxt.reserve2","reserve2")
f.RecvType3 = ProtoField.uint16("mitsubishi_mxt.RecvType3","RecvType3", nil, types)
f.reserve3 = ProtoField.uint16("mitsubishi_mxt.reserve3","reserve3")


function add_data_types(buffer, tree, print_type)
  if (print_type == 0 or print_type == 3) then
    for i = 0,28,4 do
      tree:add(buffer(i,4), "p" .. (i+4)/4 .. ": " .. buffer(i,4):le_int())
    end

  elseif (print_type == 1) then
    pose_lbl = {"x","y","z","a","b","c","l1","l2"}
    for i = 0,28,4 do
      local lbl_ind = (i+4)/4
      tree:add(buffer(i,4), pose_lbl[lbl_ind] .. ": " .. buffer(i,4):le_float())
    end
    tree:add(buffer(32,4), "sflg1: " .. buffer(32,4):le_uint())
    tree:add(buffer(36,4), "sflg2: " .. buffer(36,4):le_uint())

  elseif (print_type == 2) then
    for i = 0,28,4 do
      tree:add(buffer(i,4), "j" .. (i+4)/4 .. ": " .. buffer(i,4):le_float())
    end

  end
end

function add_to_subtree(field, buffer, subtree, index, len)
  local new_tree = subtree:add_le(field, buffer(index,len))
  return index+len, new_tree
end

function mitsubishi_mxt_proto.dissector(buffer,pinfo,tree)
    pinfo.cols.protocol = "MXT"
    local subtree = tree:add(mitsubishi_mxt_proto,buffer(),"Mitsubishi MXT Protocol Data " .. buffer:len() .. " bytes")
    local command = buffer(0,2):le_uint()
    local send_type = buffer(2,2):le_uint()
    local recv_type = buffer(4,2):le_uint()
    local next_ind = 0

    mt_subtree = getmetatable(subtree)
    function mt_subtree.add_le2 (t,field, buffer, index)
      subtree:add_le(field, buffer(index,2))
      return index+2
    end
    function mt_subtree.add_le4 (t,field, buffer, index)
      subtree:add_le(field, buffer(index,4))
      return index+4
    end
    pinfo.cols.info = "Command: " .. commands[command]

    next_ind = subtree:add_le2(f.Command, buffer, next_ind)
    next_ind = subtree:add_le2(f.SendType, buffer, next_ind)
    next_ind = subtree:add_le2(f.RecvType, buffer, next_ind)
    next_ind = subtree:add_le2(f.reserve, buffer, next_ind)

    local dat_subtree = subtree:add(buffer(next_ind, 40), "dat")
    local print_type = send_type
    if (command == 0) then
      print_type = recv_type
    end
    
    add_data_types(buffer(next_ind, 40), dat_subtree, print_type)

    next_ind = next_ind + 40
    next_ind = subtree:add_le2(f.SendIOType, buffer, next_ind)
    next_ind = subtree:add_le2(f.RecvIOType, buffer, next_ind)
    next_ind = subtree:add_le2(f.BitTop, buffer, next_ind)
    next_ind = subtree:add_le2(f.BitMask, buffer, next_ind)
    next_ind = subtree:add_le2(f.IoData, buffer, next_ind)
    next_ind = subtree:add_le2(f.TCount, buffer, next_ind)
    next_ind = subtree:add_le4(f.CCount, buffer, next_ind)

    next_ind = subtree:add_le2(f.RecvType1, buffer, next_ind)
    next_ind = subtree:add_le2(f.reserve1, buffer, next_ind)

    local dat1_subtree = subtree:add(buffer(next_ind,40), "dat1")
    add_data_types(buffer(next_ind,40), dat1_subtree, buffer(next_ind - 4,2):le_uint())
    next_ind = next_ind + 40

    next_ind = subtree:add_le2(f.RecvType2, buffer, next_ind)
    next_ind = subtree:add_le2(f.reserve2, buffer, next_ind)
    local dat2_subtree = subtree:add(buffer(next_ind,40), "dat2")
    add_data_types(buffer(next_ind,40), dat2_subtree, buffer(next_ind - 4,2):le_uint())
    next_ind = next_ind + 40

    next_ind = subtree:add_le2(f.RecvType3, buffer, next_ind)
    next_ind = subtree:add_le2(f.reserve3, buffer, next_ind)
    local dat3_subtree = subtree:add(buffer(next_ind,40), "dat3")
    add_data_types(buffer(next_ind,40), dat3_subtree, buffer(next_ind - 4,2):le_uint())
    next_ind = next_ind + 40
    print("Total: " .. next_ind)
end
-- load the udp.port table
udp_table = DissectorTable.get("udp.port")
-- register our protocol to handle udp port 10000
udp_table:add(10000,mitsubishi_mxt_proto)
