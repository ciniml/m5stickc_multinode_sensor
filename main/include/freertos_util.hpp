#ifndef FREERTOS_UTIL_HPP__
#define FREERTOS_UTIL_HPP__

#include <memory>
#include <type_traits>
#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "result.hpp"

namespace freertos
{
	class Mutex
	{
	private:
		StaticSemaphore_t body;
		SemaphoreHandle_t handle;
		bool is_locked;
	public:
		Mutex() : handle(nullptr), is_locked(false)
		{
			this->handle = xSemaphoreCreateMutexStatic(&this->body);
		}
		void lock()
		{
			xSemaphoreTake(this->handle, portMAX_DELAY);
			this->is_locked = true;
		}
        bool lock(TickType_t wait_ticks)
        {
            if( xSemaphoreTake(this->handle, wait_ticks) == pdFALSE ) {
                return false;
            }
            this->is_locked = true;
            return true;
        }

		void release()
		{
			this->is_locked = false;
			xSemaphoreGive(this->handle);
		}

		~Mutex()
		{
			if (this->handle != nullptr) {
				if (this->is_locked) {
					this->release();
				}
				vSemaphoreDelete(this->handle);
				this->handle = nullptr;
			}
		}
	};

	class WaitEvent
	{
	private:
		typedef std::unique_ptr< std::remove_pointer<EventGroupHandle_t>::type, decltype(&vEventGroupDelete)> HandleType;
		HandleType handle;
	public:
		WaitEvent() : handle(nullptr, vEventGroupDelete)
		{
			this->handle.reset(xEventGroupCreate());
		}
		WaitEvent(const WaitEvent&) = delete;
		WaitEvent(WaitEvent&& source) : handle(std::move(source.handle)) {}
		void clear()
		{
			xEventGroupClearBits(this->handle.get(), 1);
		}
		void set()
		{
			xEventGroupSetBits(this->handle.get(), 1);
		}
		void wait(bool clear_on_exit=false)
		{
			xEventGroupWaitBits(this->handle.get(), 1, clear_on_exit ? pdTRUE : pdFALSE, pdTRUE, portMAX_DELAY);
		}
		bool is_valid() const { return this->handle.operator bool(); }
	};

	template<typename TItem, std::size_t TLength>
	struct WaitQueueStorage
	{
		std::uint8_t body[TLength*sizeof(TItem)];
	};
	template<typename TItem>
	struct WaitQueueStorage<TItem, 0>
	{
	};

	template<typename TItem, std::size_t TLength>
	class WaitQueue
	{
	private:
		typedef std::unique_ptr< std::remove_pointer<QueueHandle_t>::type, decltype(&vQueueDelete) > HandleType;
		HandleType handle;
		WaitQueueStorage<TItem, TLength> storage;
		StaticQueue_t queue;
	public:

		WaitQueue() : handle(nullptr, &vQueueDelete)
		{
			this->handle.reset(xQueueCreateStatic(TLength, sizeof(TItem), this->storage.body, &this->queue));
		}
		void reset()
		{
			xQueueReset(this->handle.get());
		}
		std::size_t get_number_of_items()
		{
			return uxQueueMessagesWaiting(this->handle.get());
		}
		std::size_t get_free_spaces()
		{
			return uxQueueSpacesAvailable(this->handle.get());
		}
		bool is_full()
		{
			return this->get_free_spaces() != 0;
		}

		bool send(const TItem& item, TickType_t wait_ticks = portMAX_DELAY)
		{
			return xQueueSend(this->handle.get(), &item, wait_ticks) == pdTRUE;
		}
		bool receive(TItem& received_item, TickType_t wait_ticks = portMAX_DELAY)
		{
			return xQueueReceive(this->handle.get(), &received_item, wait_ticks) == pdTRUE;
		}
	};

	template<typename TLock>
	class LockGuard
	{
	private:
        bool own_lock;
		TLock& lock;
	public:
        typedef LockGuard<TLock> SelfType;

		LockGuard(TLock* lock) : own_lock(true), lock(*lock)
		{
			this->lock.lock();
		}
		LockGuard(TLock& lock) : own_lock(true), lock(lock)
		{
			this->lock.lock();
		}
        LockGuard(TLock& lock, TickType_t wait_ticks) : own_lock(false), lock(lock)
		{
			if( this->lock.lock(wait_ticks) ) {
                this->own_lock = true;
            }
		}
        LockGuard(SelfType&& rv) : own_lock(true), lock(rv.lock) 
        {
            rv.own_lock = false;
        }
        LockGuard& operator=(SelfType&& rv) 
        {
            if( this->own_lock ) {
                this->lock.release();
            }
            this->own_lock = rv.own_lock;
            this->lock = rv.lock;
            rv.own_lock = false;
            return *this;
        }
		~LockGuard()
		{
            if( this->own_lock ) {
                this->own_lock = false;
			    this->lock.release();
            }
		}

        operator bool() const { return this->own_lock; }
	};
    template<typename TLock> LockGuard<TLock> lock(TLock& lock_primitive) { return LockGuard<TLock>(lock_primitive); }
    template<typename TLock> LockGuard<TLock> lock(TLock& lock_primitive, TickType_t wait_ticks) { return LockGuard<TLock>(lock_primitive, wait_ticks); }

	template<typename TResult> class future;

	template<typename TResult>
	class promise_base
	{
	protected:
		friend class future<TResult>;

		struct SharedState
		{
			struct Deleter
			{
				void operator()(TResult* result) { result->~TResult(); }
			};
			typename std::aligned_storage<sizeof(TResult), alignof(TResult)>::type storage;
			std::unique_ptr<TResult, Deleter> result;
			WaitEvent event;
			Mutex mutex;
			bool is_completed;

			SharedState() : is_completed(false) {}

			void set_value(const TResult& value)
			{
				{
					LockGuard<Mutex>(this->mutex);
					this->result.reset(new(&this->storage) TResult(value));
					this->is_completed = true;
				}
				this->event.set();
			}
			void set_value(TResult&& value) noexcept
			{
				{
					LockGuard<Mutex>(this->mutex);
					this->result.reset(new(&this->storage) TResult(std::forward<TResult>(value)));
					this->is_completed = true;
				}
				this->event.set();
			}
			void wait()
			{
				this->event.wait();
			}
			TResult get_result()
			{
				if (!this->is_completed) {
					this->wait();
				}
				return *this->result;
			}
		};
		std::shared_ptr<SharedState> state;

		typedef promise_base<TResult> SelfType;

		promise_base() : state(new SharedState()) {}
		promise_base(const SelfType& other) = delete;
		promise_base(SelfType&& source) noexcept : state(std::move(source)) {}

	public:

		void set_value(TResult&& result)
		{
			this->state->set_value(std::forward<TResult>(result));
		}
		void set_value(const TResult& result)
		{
			this->state->set_value(result);
		}
	};

	template<typename TResult> class promise;

	template<typename TResult>
	class future
	{
		friend class promise<TResult>;
	private:
		typedef promise_base<TResult> PromiseType;
		typedef typename PromiseType::SharedState SharedState;

		std::shared_ptr<SharedState> state;
		
		future(const std::shared_ptr<SharedState>& state) : state(state) {}

	public:
		void wait() {
			this->state->wait();
		}
		TResult get() {
			return this->state->get_result();
		}
	};

	template<typename TResult>
	class promise : public promise_base<TResult>
	{
	public:
		typedef future<TResult> FutureType;
	private:
	public:
		FutureType get_future()
		{
			return FutureType(this->state);
		}
	};

	template<typename TResult> 
	class single_promise
	{
	private:
		struct Deleter
		{
			void operator()(promise<TResult>* target) { target->~promise(); }
		};
		typedef promise<TResult> PromiseType;

		typename std::aligned_storage< sizeof(PromiseType), alignof(PromiseType) >::type storage;
		std::unique_ptr< promise<TResult>, Deleter > inner;

	public:
		bool is_valid() const { return this->inner.operator bool(); }
		void reset()
		{
			this->inner.reset(new (&this->storage) PromiseType());
		}
		void set_value(TResult&& value) {
			this->inner->set_value(std::forward<TResult>(value));
		}
		void set_value(const TResult& value) {
			this->inner->set_value(value);
		}
		future<TResult> get_future() { return this->inner->get_future(); }
	};

	struct Task
	{
		TaskHandle_t handle;
		bool own_handle;
		
		Task(TaskHandle_t handle) : handle(handle), own_handle(false) {}
		Task(TaskHandle_t handle, bool own_handle) : handle(handle), own_handle(own_handle) {}
		Task(const Task& task) : handle(task.handle), own_handle(false) {}
		Task(Task&& task) : handle(task.handle), own_handle(task.own_handle) 
		{
			task.own_handle = false;
		}

		~Task() {
			if( this->own_handle && this->handle != nullptr ) {
				vTaskDelete(this->handle);
				this->handle = nullptr;
				this->own_handle = false;
			}
		}

		struct NotifyFromISRResut
		{
			bool success;
			bool higher_priority_task_woken;
		};

		operator TaskHandle_t() const { return this->handle; }
		operator bool() const { return this->handle != nullptr; }

		bool notify(std::uint32_t value, eNotifyAction action)
		{
			return xTaskNotify(this->handle, value, action) == pdPASS;
		}
		Result<bool, bool> notify_from_isr(std::uint32_t value, eNotifyAction action)
		{
			BaseType_t higher_priority_task_woken = pdFALSE;
			if( xTaskNotifyFromISR(this->handle, value, action, &higher_priority_task_woken) == pdTRUE ) {
				return success(higher_priority_task_woken == pdTRUE);
			}
			else {
				return failure(true);
			}
		}

		static Task current()
		{
			return Task(xTaskGetCurrentTaskHandle());
		}

		static Result<std::uint32_t, bool> notify_wait(std::uint32_t bits_to_clear_on_entry, std::uint32_t bits_to_clear_on_exit, TickType_t ticks_to_wait)
		{
			std::uint32_t notification_value = 0;
			auto result = xTaskNotifyWait(bits_to_clear_on_entry, bits_to_clear_on_exit, &notification_value, ticks_to_wait);
			if( result != pdPASS ) {
				return failure(true);
			}
			else {
				return success<std::uint32_t>(notification_value);
			}
		}
	};


	template<std::uint32_t StackSize, typename TaskFunc>
	struct StaticTask 
	{
		typedef StaticTask<StackSize, TaskFunc> SelfType;

		StackType_t stack[StackSize];
		TaskHandle_t handle;
		StaticTask_t context;
		TaskFunc task_func;

		static void task_proc_wrapper(void* parameters) 
		{
			auto this_ = reinterpret_cast<SelfType*>(parameters);
			this_->task_func();
		}

		StaticTask(TaskFunc&& task_func) : handle(nullptr), task_func(std::forward<TaskFunc>(task_func)) {}

		operator bool() const { return this->handle != nullptr; }

		bool start(const char* name, UBaseType_t priority, BaseType_t core_id) 
		{
			this->handle = xTaskCreateStaticPinnedToCore(&StaticTask::task_proc_wrapper, name, StackSize, this, priority, this->stack, &this->context, core_id);
			return this->handle != nullptr;
		}

		Task task() const { return Task(this->handle); }
	};
};


#endif //FREERTOS_UTIL_HPP__
