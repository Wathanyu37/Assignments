

for k = 1:4:size(B_L,1)
            for i=ix(k)
                for j=iy(k) 
                    if ismember([i,j],B_L(1:end-1,:),'rows') == 1
                        
                       if C(j+y_offset,i+x_offset) == l_0
                       C(j+y_offset,i+x_offset) = C(j+y_offset,i+x_offset) + l_free - C(j+y_offset,i+x_offset);%l_0;
                       end
                       
                    elseif ismember([i,j],B_L(end,:),'rows') == 1 
                       %if C(j+y_offset,i+x_offset) == l_0
                        C(j+y_offset,i+x_offset) = C(j+y_offset,i+x_offset) + l_occ - C(j+y_offset,i+x_offset);%l_0;
                       %end
                    end
                end        
            end
end
        
        