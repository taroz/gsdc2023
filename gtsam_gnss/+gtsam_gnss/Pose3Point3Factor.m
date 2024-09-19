%class Pose3Point3Factor, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose3Point3Factor(size_t keyP, size_t keyX, Base model)
%
classdef Pose3Point3Factor < gtsam.NoiseModelFactor
  properties
    ptr_gtsam_gnssPose3Point3Factor = 0
  end
  methods
    function obj = Pose3Point3Factor(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_gnss_wrapper(37, varargin{2});
        end
        base_ptr = gtsam_gnss_wrapper(36, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gtsam_gnss_wrapper(38, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gtsam_gnss.Pose3Point3Factor constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsam_gnssPose3Point3Factor = my_ptr;
    end

    function delete(obj)
      gtsam_gnss_wrapper(39, obj.ptr_gtsam_gnssPose3Point3Factor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
