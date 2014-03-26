function p = prio(link, slot)
   seed = bitshift((bitshift(link(:, 1), 8) + link(:, 2)), 16) + slot;
   p = rand32(seed);

   function r = rand32(seed)
       r = mod(seed * 279470273, 4294967291);
       r = floor(r);
   end
end