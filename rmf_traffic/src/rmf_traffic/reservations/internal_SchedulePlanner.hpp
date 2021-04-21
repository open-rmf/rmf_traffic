#include "internal_DatabasImpl.hpp"

namespace rmf_traffic {
namespace reservations {

class ResourcePlannerGenerator
{
    using Schedule = Database::Implementation::ResourceSchedule;
    std::unordered_map<ReservationId, std::optional<Time>> earliest_start_times;
    std::unordered_map<ReservationId, std::optional<Time>> latest_start_times;
    const Schedule sched;

    class PlanGeneratorIterator
    {
    public:
        bool can_progress_prev =true;
        bool can_progress_next =true;
    private:
        Schedule::const_iterator _iter, _prev_checked, _next_checked;
        const ReservationRequest _req;
        Duration last_push_back{0}, last_push_forward{0};
        const Schedule _sched;

        PlanGeneratorIterator(Schedule::const_iterator iter,
            const Schedule& schedule,
            const ReservationRequest& req,
            const Duration& duration):
            _sched(schedule),
            _req(req)
        {
            _iter = iter;
            if(_iter == _sched.begin())
            {
                can_progress_prev = false;
                _next_checked = iter;
                //while(_next_checked->first > )
                //{
                //
                //}
            }
            _prev_checked = std::prev(iter); 
            _next_checked = iter;
            auto gap = _next_checked->first - _prev_checked->first;

        }

        PlanGeneratorIterator(bool)
        {
            can_progress_next = false;
            can_progress_prev = false;
        }
    friend ResourcePlannerGenerator;
    public:
        std::optional<Database::Implementation::Plan>
            next_insertion_slot()
        {
            if(!can_progress_prev && !can_progress_next)
            {
                return std::nullopt;
            }

            if(can_progress_prev && can_progress_next)
            {
              
            }
        }
    
    };
    
    ResourcePlannerGenerator(const Schedule& sched): _sched(sched)
    {
        
    }

    ResourcePlannerGenerator get_insertion_points(Schedule::const_iterator 
        insertion_point, ReservationRequest req)
    {
        if(req.is_indefinite())
        {
            if(insertion_point != sched.end())
            {
                return ResourcePlannerGenerator
            }
        }
    }
}
}
}